#include "real_deepracer_imu_sensor.h"

CRealDeepracerIMUSensor::CRealDeepracerIMUSensor(const std::shared_ptr<rclcpp::Node>& pt_node_handle) : CRealDeepracerDevice(pt_node_handle){
    m_ptImuSubscription = pt_node_handle->create_subscription<sensor_msgs::msg::Imu>(
            //http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
            "/imu_pkg/data_raw",
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
    /*
        As of 06/28/2024, the signs of the ang. vel and lin. acc. were obtained after manually
        calibrating to point to:
            x = forward,
            y = leftward, and
            z = upward of the robot,
        as well as matching right-handed rotations of each axis.

        This adjustment is dependent on how the `imu_node.py` publishes them (see
        https://github.com/larsll/larsll-deepracer-imu-pkg/blob/a68101a2e0ff78b9272b3de4f1a73e6026d6b8d8/imu_pkg/imu_pkg/imu_node.py#L191)
        If later changes are pushed to that repository and subsequently updates the way the
        aws-deepracer node runs, the signs may not stay as calibrated.
    */

    // Update the angular velocity values
    m_vecAngularVelocity.SetX(msg->angular_velocity.x);
    m_vecAngularVelocity.SetY(msg->angular_velocity.y);
    m_vecAngularVelocity.SetZ(msg->angular_velocity.z);

    //Update the angular velocity values
    m_vecLinearAcceleration.SetX(-msg->linear_acceleration.x);
    m_vecLinearAcceleration.SetY(-msg->linear_acceleration.y);
    m_vecLinearAcceleration.SetZ(-msg->linear_acceleration.z);

    // Populate as reading struct
    m_sReading.AngVelocity = m_vecAngularVelocity;
    m_sReading.LinAcceleration = m_vecLinearAcceleration;
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