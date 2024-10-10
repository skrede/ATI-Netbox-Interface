#include "sensor_interface/sensorcontroller.h"

#include <iostream>

using namespace estimation::sensor_interface;

SensorController::SensorController(const std::string &hostname, uint32_t port)
: m_port(port)
, m_hostname(hostname)
, m_sensor_connected(false)
, m_count_per_force(1000000u)
, m_count_per_torque(1000000u)
{
    SensorSnapshot s;
    s.fx = 0.0;
    s.fy = 0.0;
    s.fz = 0.0;
    s.tx = 0.0;
    s.ty = 0.0;
    s.tz = 0.0;
    m_sensor_load_snapshot.store(s);
    startSensorInterface();
}

bool SensorController::hasConnectedSensor()
{
    return m_sensor_connected;
}

void SensorController::setCalibrationBias(const Eigen::Vector3d &force_bias, const Eigen::Vector3d &torque_bias)
{
    std::unique_lock<std::shared_mutex> l(m_bias_lock);
    m_force_bias = force_bias;
    m_torque_bias = torque_bias;
}

void SensorController::setCountPerForceTorque(uint32_t fcount, uint32_t tcount)
{
    m_count_per_force = fcount;
    m_count_per_torque = tcount;
}

Eigen::Vector3d SensorController::forceBias() const
{
    std::shared_lock<std::shared_mutex> l(m_bias_lock);
    return m_force_bias;
}

Eigen::Vector3d SensorController::torqueBias() const
{
    std::shared_lock<std::shared_mutex> l(m_bias_lock);
    return m_torque_bias;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SensorController::currentRawLoad()
{
    auto data = m_sensor_load_snapshot.load();
    Eigen::Vector3d force(data.fx, data.fy, data.fz);
    Eigen::Vector3d torque(data.tx, data.ty, data.tz);
    return std::make_pair(force, torque);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> SensorController::currentUnbiasedLoad()
{
    auto data = m_sensor_load_snapshot.load();
    Eigen::Vector3d force(data.fx, data.fy, data.fz);
    Eigen::Vector3d torque(data.tx, data.ty, data.tz);
    std::shared_lock<std::shared_mutex> l(m_bias_lock);
    return std::make_pair(force - m_force_bias, torque - m_torque_bias);
}

void SensorController::addSensorReadingReceivedListener(SensorController::SensorReadingListener listener)
{
    std::lock_guard<std::mutex> l(m_listener_lock);
    m_listeners.push_back(listener);
}

void SensorController::sensorLoadReceived(int32_t fx, int32_t fy, int32_t fz, int32_t tx, int32_t ty, int32_t tz)
{
    double cpf = m_count_per_force.load();
    double cpt = m_count_per_torque.load();
    Eigen::ArrayXd m(6);
    m(0) = (double)fx / cpf;
    m(1) = (double)fy / cpf;
    m(2) = (double)fz / cpf;
    m(3) = (double)tx / cpt;
    m(4) = (double)ty / cpt;
    m(5) = (double)tz / cpt;
    Eigen::Vector3d f(m.head(3));
    Eigen::Vector3d t(m.tail(3));
    m_sensor_load_snapshot =
    {
        f.x(),
        f.y(),
        f.z(),
        t.x(),
        t.y(),
        t.z()
    };
    {
        std::lock_guard<std::mutex> l(m_listener_lock);
        if(m_listeners.empty())
            return;
        for(const auto &listener : m_listeners)
            listener(f, t);
    }
}

void SensorController::startSensorInterface()
{
    std::unique_lock<std::shared_mutex> l(m_interface_lock);
    if(m_netbox_rdt)
    {
        if(m_netbox_rdt->isStreaming())
            m_netbox_rdt->stopStreaming();
        m_netbox_rdt.reset();
    }
    m_netbox_rdt = std::make_unique<NetboxRdtClient>();
    auto cb = std::bind(&SensorController::sensorLoadReceived, this, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
    m_netbox_rdt->setSensorLoadListener(cb);
    m_netbox_rdt->startStreaming(m_hostname, m_port);
    m_sensor_connected = true;
}
