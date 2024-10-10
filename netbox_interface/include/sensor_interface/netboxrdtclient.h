#ifndef ESTIMATION_SENSOR_INTERFACE_NETBOXRDTIOSERVER_H
#define ESTIMATION_SENSOR_INTERFACE_NETBOXRDTIOSERVER_H

#include "simple_socket/UDPSocket.hpp"

#include <string>

#include <atomic>
#include <thread>
#include <functional>

namespace estimation::sensor_interface {
enum class RTDCommand : uint16_t
{
    HEADER                           = 0x1234,
    STOP_STREAM                      = 0x0000,
    START_HIGH_SPEED_REALTIME_STREAM = 0x0002,
    START_HIGH_SPEED_BUFFERED_STREAM = 0x0003,
    START_MULTI_UNIT_STREAMING       = 0x0004,
    RESET_THRESHOLD_LATCH            = 0x0041,
    SET_SOFTWARE_BIAS                = 0x0042
};

struct RTDRequest
{
    RTDRequest(RTDCommand command) : RTDRequest(command, 0)
    {
    }

    RTDRequest(RTDCommand command, uint32_t sample_count)
    : header(static_cast<uint16_t>(RTDCommand::HEADER))
    , command(static_cast<uint16_t>(command))
    , sample_count(sample_count)
    {
    }

    uint16_t header;
    uint16_t command;
    uint32_t sample_count = 0;
};

struct RTDResponse
{
    uint32_t rdt_package_sequence_index;
    uint32_t ft_internal_sequence_index;
    uint32_t status;
    int32_t fx;
    int32_t fy;
    int32_t fz;
    int32_t tx;
    int32_t ty;
    int32_t tz;
};

typedef std::function<void (int32_t fx, int32_t fy, int32_t fz, int32_t tx, int32_t ty, int32_t tz)> FTSensorLoadListener;

class NetboxRdtClient
{
public:
    NetboxRdtClient();
    ~NetboxRdtClient();

    void startStreaming(const std::string &netbox_ip, uint16_t netbox_port);
    bool isStreaming() const;
    void stopStreaming();

    void setSensorLoadListener(FTSensorLoadListener listener);

private:
    std::thread m_worker;
    std::atomic<bool> m_connected;
    std::atomic<bool> m_streaming;
    FTSensorLoadListener m_load_listener;
    std::unique_ptr<simple_socket::UDPSocket> m_client;
    std::unique_ptr<simple_socket::SimpleConnection> m_client_connection;

    void connectedChanged(bool connected);

    std::string serialize(const RTDRequest &request);

    void receiveMessage(const std::string &payload);
    RTDResponse deserialize(const std::string &response);
};
}

#endif
