#ifndef ESTIMATION_SENSOR_INTERFACE_SENSORCONTROLLER_H
#define ESTIMATION_SENSOR_INTERFACE_SENSORCONTROLLER_H

#include "sensor_interface/netboxrdtclient.h"

#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <shared_mutex>

#include <Eigen/Core>

namespace estimation::sensor_interface {
class SensorController
{
    struct SensorSnapshot
    {
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;
    };

public:
    typedef std::function<void(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)> SensorReadingListener;

    SensorController(const std::string &hostname, uint32_t port);

    bool hasConnectedSensor();

    void setCalibrationBias(const Eigen::Vector3d &force_bias, const Eigen::Vector3d &torque_bias);

    void setCountPerForceTorque(uint32_t fcount, uint32_t tcount);

    Eigen::Vector3d forceBias() const;
    Eigen::Vector3d torqueBias() const;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> currentRawLoad();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> currentUnbiasedLoad();

    void addSensorReadingReceivedListener(SensorReadingListener listener);

private:
    uint32_t m_port;
    std::string m_hostname;
    std::mutex m_listener_lock;
    Eigen::Vector3d m_force_bias;
    Eigen::Vector3d m_torque_bias;
    std::shared_mutex m_interface_lock;
    std::atomic<bool> m_sensor_connected;
    mutable std::shared_mutex m_bias_lock;
    std::atomic<uint32_t> m_count_per_force;
    std::atomic<uint32_t> m_count_per_torque;
    std::atomic<SensorSnapshot> m_sensor_load_snapshot;
    std::unique_ptr<NetboxRdtClient> m_netbox_rdt;
    std::vector<SensorReadingListener> m_listeners;

    void sensorLoadReceived(int32_t fx, int32_t fy, int32_t fz, int32_t tx, int32_t ty, int32_t tz);

    void startSensorInterface();
};
}

#endif
