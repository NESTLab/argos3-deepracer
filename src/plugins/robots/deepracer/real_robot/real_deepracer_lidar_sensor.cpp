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
//    m_vecRanges = std::vector<double> doubleVec(ranges.begin(), ranges.end());
    m_fAngle_min = msg->angle_min;
    m_fAngle_max = msg->angle_max;
    m_fAngle_increment = msg->angle_increment;
    m_fTime_increment = msg->time_increment;
    m_fScan_time = msg->scan_time;
    m_fRange_min = msg->range_min;
    m_fRange_max = msg->range_max;

}

/****************************************/
/****************************************/


void CRealDeepracerLIDARSensor::Do(Real f_elapsed_time) {
    //Takes the saved values and store them all in one buffer
    //Should I store the timing as well ?
}

/****************************************/
/****************************************/

CVector3 CRealDeepracerLIDARSensor::GetReading() const {
    return m_vecRanges;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetAngleMin() {
    return m_fAngle_min;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetAngleMax() {
    return m_fAngle_max;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetAngleIncrement() {
    return m_fAngle_increment;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetRangeMin() {
    return m_fRange_min;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetRangeMax() {
    return m_fRange_max;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetTimeIncrement() {
    return m_fTime_increment;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetTimeScan() {
    return m_fScan_time;
}