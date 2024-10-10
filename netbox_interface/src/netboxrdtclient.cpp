#include "sensor_interface/netboxrdtclient.h"

#include <functional>

#include <winsock.h>

#include <stdexcept>

using namespace estimation::sensor_interface;

NetboxRdtClient::NetboxRdtClient()
: m_connected(false)
, m_streaming(false)
{
}

NetboxRdtClient::~NetboxRdtClient()
{
    stopStreaming();
}

void NetboxRit adddtClient::startStreaming(const std::string &netbox_ip, uint16_t netbox_port)
{
    m_client = std::make_unique<simple_socket::UDPSocket>(netbox_port);
    m_client_connection = m_client->makeConnection(netbox_ip, netbox_port);
    if(m_client_connection == nullptr)
        throw std::runtime_error("Unable to connect to ATI Netbox on " + netbox_ip + ":" + std::to_string(netbox_port));
    RTDRequest request
    {
        RTDCommand::START_HIGH_SPEED_REALTIME_STREAM
    };
    auto data = serialize(request);
    m_client_connection->write(data.data(), data.size());
    m_streaming = true;
    m_worker = std::thread([&]()
    {
        unsigned char buffer[2048];
        while(m_streaming)
        {
            auto read = m_client_connection->read(buffer, 2048);
            std::string payload(buffer, buffer + read);
            receiveMessage(payload);
        }
    });
}

bool NetboxRdtClient::isStreaming() const
{
    return m_streaming;
}

void NetboxRdtClient::stopStreaming()
{
    if(!m_streaming)
        return;
    RTDRequest request
    {
        RTDCommand::STOP_STREAM
    };
    auto data = serialize(request);
    m_client_connection->write(data.data(), data.size());
    m_streaming = false;
    m_worker.join();
}

void NetboxRdtClient::setSensorLoadListener(FTSensorLoadListener listener)
{
    m_load_listener = listener;
}

void NetboxRdtClient::connectedChanged(bool connected)
{
    m_connected = connected;
}

std::string NetboxRdtClient::serialize(const RTDRequest &request)
{
    std::string buffer;
    buffer.resize(8);
    auto raw_buf = buffer.data();
    *(reinterpret_cast<uint16_t*>(&raw_buf[0])) = htons(0x1234);
    *(reinterpret_cast<uint16_t*>(&raw_buf[2])) = htons(request.command);
    *(reinterpret_cast<uint32_t*>(&raw_buf[4])) = htonl(request.sample_count);
    return buffer;
}

void NetboxRdtClient::receiveMessage(const std::string &payload)
{
    auto response = deserialize(payload);
    m_load_listener
    (
        response.fx,
        response.fy,
        response.fz,
        response.tx,
        response.ty,
        response.tz
    );
}

RTDResponse NetboxRdtClient::deserialize(const std::string &response)
{
    RTDResponse ret;
    auto raw_buf = response.data();
    ret.rdt_package_sequence_index = ntohl(*(reinterpret_cast<const uint32_t*>(&raw_buf[0])));
    ret.ft_internal_sequence_index = ntohl(*(reinterpret_cast<const uint32_t*>(&raw_buf[4])));
    ret.status = ntohl(*(reinterpret_cast<const uint32_t*>(&raw_buf[8])));
    ret.fx = ntohl(*(reinterpret_cast<const int32_t*>(&raw_buf[12])));
    ret.fy = ntohl(*(reinterpret_cast<const int32_t*>(&raw_buf[16])));
    ret.fz = ntohl(*(reinterpret_cast<const int32_t*>(&raw_buf[20])));
    ret.tx = ntohl(*(reinterpret_cast<const int32_t*>(&raw_buf[24])));
    ret.ty = ntohl(*(reinterpret_cast<const int32_t*>(&raw_buf[28])));
    ret.tz = ntohl(*(reinterpret_cast<const int32_t*>(&raw_buf[32])));
    return ret;
}
