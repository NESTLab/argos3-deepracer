#include "real_deepracer_imu_sensor.h"

CRealDeepracerIMUSensor::CRealDeepracerIMUSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle) : CRealDeepracerDevice(*pt_node_handle){
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
    m_vecOrientation.push_back(msg->orientation.x);
    m_vecOrientation.push_back(msg->orientation.y);
    m_vecOrientation.push_back(msg->orientation.z);
    m_vecOrientation.push_back(msg->orientation.w);
}

/****************************************/
/****************************************/


void CRealDeepracerIMUSensor::Do(Real f_elapsed_time) {
    //Takes the saved values and store them all in one buffer
    //Should I store the timing as well ?
    m_vecOrientationList.push_back(m_vecOrientation);
    m_vecAngularVelocityList.push_back(m_vecAngularVelocity);
    m_vecLinearAccelerationList.push_back(m_vecLinearAcceleration);
}

/****************************************/
/****************************************/

std::vector<float> CRealDeepracerIMUSensor::GetCurrentOrientation() {
    return m_vecOrientation;
}