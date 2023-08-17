#ifndef REAL_DEEPRACER_LIDAR_SENSOR_H
#define REAL_DEEPRACER_LIDAR_SENSOR_H
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_lidar_sensor.h>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer_device.h>

#include <functional>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>

using namespace argos;

class CRealDeepracerLIDARSensor : public CCI_DeepracerLIDARSensor,
                                  public CRealDeepracerDevice {
public:

    CRealDeepracerLIDARSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle);

    virtual ~CRealDeepracerLIDARSensor();

    virtual void Do(Real f_elapsed_time);

    /**
     * Returns the range data [m] (Note: values < range_min or > range_max should be discarded)
     */
    virtual Real GetReading(UInt32 un_idx) const;

    /**
     * Returns the readings of this sensor
     */
    inline size_t GetNumReadings() const {
        return Abs(m_fAngleMax - m_fAngleMin) / m_fAngleIncrement;
    }

    /*
     * Switches the sensor power on.
     */
    virtual void PowerOn();

    /*
     * Switches the sensor power off.
     */
    virtual void PowerOff();

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
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr              m_ptLidarStartClient;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr              m_ptLidarStopClient;

    std::vector<Real> m_vecRanges; // Vector of ranges

    Real m_fAngleMin;       // Start angle of the scan [rad]
    Real m_fAngleMax;       // End angle of the scan [rad]
    Real m_fAngleIncrement; // Angular distance between measurements [rad]

    Real m_fTimeIncrement; // Time between measurements [seconds]
    Real m_fScanTime;      // Mime between scans [seconds]

    Real m_fRangeMin; // Minimum range value [m]
    Real m_fRangeMax; // Maximum range value [m]
};
#endif
