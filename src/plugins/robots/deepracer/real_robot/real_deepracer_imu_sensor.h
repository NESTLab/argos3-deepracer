#ifndef REAL_DEEPRACER_IMU_SENSOR_H
#define REAL_DEEPRACER_IMU_SENSOR_H

#include <sensor_msgs/msg/imu.hpp>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer_device.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_imu_sensor.h>
#include <argos3/core/utility/math/vector3.h>


using namespace argos;
class CRealDeepracerIMUSensor :
        public CCI_DeepracerIMUSensor,
        public CRealDeepracerDevice {
public:
    CRealDeepracerIMUSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle);
    virtual ~CRealDeepracerIMUSensor();
    virtual void Do(Real f_elapsed_time);

    virtual CVector3 GetAngularVelocities() const;
    virtual CVector3 GetLinearAccelerations() const;

private:
    void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_ptImuSubscription;
    CVector3 m_vecAngularVelocity;
    CVector3 m_vecLinearAcceleration;

};


#endif
