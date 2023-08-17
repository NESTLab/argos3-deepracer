#include "real_deepracer_lidar_sensor.h"

CRealDeepracerLIDARSensor::CRealDeepracerLIDARSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle) : CRealDeepracerDevice(*pt_node_handle) {
    m_ptLidarSubscription = pt_node_handle->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(
            &CRealDeepracerLIDARSensor::LidarCallback,
            this,
            std::placeholders::_1));
}

CRealDeepracerLIDARSensor::~CRealDeepracerLIDARSensor() {
}

/****************************************/
/****************************************/

void CRealDeepracerLIDARSensor::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    m_vecRanges       = std::vector<Real>{msg->ranges.begin(), msg->ranges.end()};
    m_fAngleMin       = msg->angle_min;
    m_fAngleMax       = msg->angle_max;
    m_fAngleIncrement = msg->angle_increment;
    m_fTimeIncrement  = msg->time_increment;
    m_fScanTime       = msg->scan_time;
    m_fRangeMin       = msg->range_min;
    m_fRangeMax       = msg->range_max;
}

/****************************************/
/****************************************/

void CRealDeepracerLIDARSensor::Do(Real f_elapsed_time) {
    // Do nothing, the callback handles data population
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetReading(UInt32 un_idx) const {
    return m_vecRanges.at(un_idx);
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetAngleMin() {
    return m_fAngleMin;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetAngleMax() {
    return m_fAngleMax;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetAngleIncrement() {
    return m_fAngleIncrement;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetRangeMin() {
    return m_fRangeMin;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetRangeMax() {
    return m_fRangeMax;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetTimeIncrement() {
    return m_fTimeIncrement;
}

/****************************************/
/****************************************/

Real CRealDeepracerLIDARSensor::GetTimeScan() {
    return m_fScanTime;
}