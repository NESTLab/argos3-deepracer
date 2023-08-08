#ifndef REAL_DEEPRACER_LIDAR_SENSOR_H
#define REAL_DEEPRACER_LIDAR_SENSOR_H
#include <vector>
#include <memory>
#include <functional>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer_device.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_lidar_sensor.h>

using namespace argos;

class CRealDeepracerLIDARSensor :
        public CCI_DeepracerLIDARSensor,
        public CRealDeepracerDevice {
public:
    CRealDeepracerLIDARSensor(const std::shared_ptr<CRealDeepracer>& pt_node_handle);
    virtual ~CRealDeepracerLIDARSensor();
    virtual void Do(Real f_elapsed_time);
    std::vector<std::vector<float>> GetReadings();
    size_t GetNumReadings();

    /**
     * Returns the readings of this sensor
     */
    virtual long GetReading(UInt32 un_idx) const {};

    /**
     * Returns the readings of this sensor
     */
    virtual size_t GetNumReadings() const {};

    /**
     * Switches the sensor power on.
     */
    virtual void PowerOn() {};

    /*
     * Switches the sensor power off.
     */
    virtual void PowerOff() {};

    /*
     * Switches the laser on.
     */
    virtual void LaserOn() {};

    /*
     * Switches the laser off.
     */
    virtual void LaserOff() {};
private:
    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_ptLidarSubscription;
    std::vector<float> m_vecIntensities;
    std::vector<std::vector<float>> m_vecIntensitiesList;

};
#endif
