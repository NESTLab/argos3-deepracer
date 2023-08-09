#ifndef REAL_DEEPRACER_IMU_SENSOR_H
#define REAL_DEEPRACER_IMU_SENSOR_H

#include <sensor_msgs/msg/imu.hpp>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer_device.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_imu_sensor.h>


using namespace argos;
class CRealDeepracerIMUSensor :
        public CCI_DeepracerIMUSensor,
        public CRealDeepracerDevice{
public:
    CRealDeepracerIMUSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle);
    virtual ~CRealDeepracerIMUSensor();
    virtual void Do(Real f_elapsed_time);

    virtual std::vector<Real> GetOrientations() const{};
    std::vector<float> GetCurrentOrientation();
    virtual std::vector<Real> GetAngularVelocities() const {};
    virtual std::vector<Real> GetLinearAccelerations() const {};

private:
    void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_ptImuSubscription;
    std::vector<float> m_vecOrientation;
    std::vector<float> m_vecAngularVelocity;
    std::vector<float> m_vecLinearAcceleration;
    std::vector<std::vector<float>> m_vecOrientationList;
    std::vector<std::vector<float>> m_vecAngularVelocityList;
    std::vector<std::vector<float>> m_vecLinearAccelerationList;

};


#endif
