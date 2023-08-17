#include "real_deepracer_imu_sensor.h"

CRealDeepracerIMUSensor::CRealDeepracerIMUSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle) : CRealDeepracerDevice(pt_node_handle){
    m_ptImuSubscription = pt_node_handle->create_subscription<sensor_msgs::msg::Imu>(
            //http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
            "/imu_pkg/raw_data",
            10,
            std::bind(
                    &CRealDeepracerIMUSensor::ImuCallBack,
                    this,
                    std::placeholders::_1
            )
    );
}

CRealDeepracerIMUSensor::~CRealDeepracerIMUSensor() {
}

/****************************************/
/****************************************/

void CRealDeepracerIMUSensor::ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    //Update the angular velocity values
    m_vecAngularVelocity.SetX(msg->angular_velocity.x);
    m_vecAngularVelocity.SetY(msg->angular_velocity.y);
    m_vecAngularVelocity.SetZ(msg->angular_velocity.z);

    //Update the angular velocity values
    m_vecLinearAcceleration.SetX(msg->linear_acceleration.x);
    m_vecLinearAcceleration.SetY(msg->linear_acceleration.y);
    m_vecLinearAcceleration.SetZ(msg->linear_acceleration.z);

}

/****************************************/
/****************************************/


void CRealDeepracerIMUSensor::Do(Real f_elapsed_time) {
}



/****************************************/
/****************************************/

CVector3 CRealDeepracerIMUSensor::GetAngularVelocities() const{
    return m_vecAngularVelocity;
}

/****************************************/
/****************************************/

CVector3 CRealDeepracerIMUSensor::GetLinearAccelerations() const{
    return m_vecLinearAcceleration;
}