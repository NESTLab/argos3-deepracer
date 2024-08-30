#include "real_deepracer_lidar_sensor.h"

CRealDeepracerLIDARSensor::CRealDeepracerLIDARSensor(const std::shared_ptr<rclcpp::Node>& pt_node_handle) : CRealDeepracerDevice(pt_node_handle) {
    m_ptLidarSubscription = pt_node_handle->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(
            &CRealDeepracerLIDARSensor::LidarCallback,
            this,
            std::placeholders::_1));

    m_ptLidarStartClient = pt_node_handle->create_client<std_srvs::srv::Empty>("start_motor");
    m_ptLidarStopClient  = pt_node_handle->create_client<std_srvs::srv::Empty>("stop_motor");
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

void CRealDeepracerLIDARSensor::PowerOn() {
    std::shared_ptr<std_srvs::srv::Empty::Request> ptRequest;

    auto ptResult = m_ptLidarStartClient->async_send_request(ptRequest);

    if (rclcpp::spin_until_future_complete(m_ptNodeHandle, ptResult) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        LOG << "LIDAR motor started" << std::endl;
    } else {
        LOGERR << "Failed to call ROS service 'start_motor'" << std::endl;
    }
}

/****************************************/
/****************************************/

void CRealDeepracerLIDARSensor::PowerOff() {
    std::shared_ptr<std_srvs::srv::Empty::Request> ptRequest;

    auto ptResult = m_ptLidarStopClient->async_send_request(ptRequest);

    if (rclcpp::spin_until_future_complete(m_ptNodeHandle, ptResult) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        LOG << "LIDAR motor stopped" << std::endl;
    } else {
        LOGERR << "Failed to call ROS service 'stop_motor'" << std::endl;
    }
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
