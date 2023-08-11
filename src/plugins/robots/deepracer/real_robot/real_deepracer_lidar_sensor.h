#ifndef REAL_DEEPRACER_LIDAR_SENSOR_H
#define REAL_DEEPRACER_LIDAR_SENSOR_H
#include <vector>
#include <memory>
#include <functional>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer_device.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_lidar_sensor.h>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer.h>
#include <argos3/core/utility/math/vector3.h>


using namespace argos;


class CRealDeepracerLIDARSensor :
        public CCI_DeepracerLIDARSensor,
        public CRealDeepracerDevice {
public:
    CRealDeepracerLIDARSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle);
    virtual ~CRealDeepracerLIDARSensor();
    virtual void Do(Real f_elapsed_time);

    /**
     * Returns the range data [m] (Note: values < range_min or > range_max should be discarded)
     */
    virtual CVector3 GetReading() const;
    
    /**
     * Returns start angle of the scan [rad]
     */
    Real GetAngleMin();

    /**
     * Returns end angle of the scan [rad]
     */
    Real GetAngleMax();

    /**
     * Returns end angle of the scan [rad]
     */
    Real GetAngleIncrement();

    /**
     * Returns minimum range value [m]
     */
    Real GetRangeMin();

    /**
     * Returns maximum range value [m]
     */
    Real GetRangeMax();

    /**
     * Returns time between measurements [seconds]
     */
    Real GetTimeIncrement();

    /**
     * Returns time between scans [seconds]
     */
    Real GetTimeScan();

private:
    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_ptLidarSubscription;
    CVector3 m_vecRanges;

    Real m_fAngle_min;        // Start angle of the scan [rad]
    Real m_fAngle_max;        // End angle of the scan [rad]
    Real m_fAngle_increment;  // Angular distance between measurements [rad]

    Real m_fTime_increment;   // Time between measurements [seconds]
    Real m_fScan_time;        // Mime between scans [seconds]

    Real m_fRange_min;        // Minimum range value [m]
    Real m_fRange_max;        // Maximum range value [m]

};
#endif
