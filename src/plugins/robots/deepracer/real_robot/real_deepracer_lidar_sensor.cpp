#include "real_deepracer_lidar_sensor.h"

CRealDeepracerLIDARSensor::CRealDeepracerLIDARSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle) : CRealDeepracerDevice(*pt_node_handle){
    m_ptLidarSubscription = pt_node_handle->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(
                    &CRealDeepracerLIDARSensor::LidarCallback,
                    this,
                    std::placeholders::_1
                )
            );
}

CRealDeepracerLIDARSensor::~CRealDeepracerLIDARSensor() {
}

/****************************************/
/****************************************/

void CRealDeepracerLIDARSensor::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    //Updates the values by putting them in m_nEncoder(s)
    //Is it okay for the msg to be a shared pointer?
//    RCLCPP_INFO(m_tNodeHandle.get_logger(), "LiDAR's intensities received: '%s'", msg->intensities.c_str());
    m_vecIntensities = msg->ranges;
}

/****************************************/
/****************************************/


void CRealDeepracerLIDARSensor::Do(Real f_elapsed_time) {
    //Takes the saved values and store them all in one buffer
    //Should I store the timing as well ?
    m_vecIntensitiesList.push_back(m_vecIntensities);

}

/****************************************/
/****************************************/

std::vector<std::vector<float>>CRealDeepracerLIDARSensor::GetReadings() {
    //Return the updated values
    return m_vecIntensitiesList;
}

/****************************************/
/****************************************/

size_t CRealDeepracerLIDARSensor::GetNumReadings() {
    return m_vecIntensitiesList.size();
}