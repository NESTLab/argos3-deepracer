#ifndef REAL_DEEPRACER_H
#define REAL_DEEPRACER_H

#include <argos3/core/utility/rate.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/real_robot/real_robot.h>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer_device.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

using namespace argos;

class CRealDeepracerDevice; // forward declare CRealDeepracerDevice

class CRealDeepracer :
        public CRealRobot,
        public rclcpp::Node,
        public std::enable_shared_from_this<CRealDeepracer> {
public:

    CRealDeepracer();
    virtual ~CRealDeepracer() {}
    virtual void InitRobot(std::string str_argos_fname, std::string str_controller_id);
    virtual void Destroy();
    virtual CCI_Actuator* MakeActuator(const std::string& str_name);
    virtual CCI_Sensor* MakeSensor(const std::string& str_name);
    virtual void Sense(Real f_elapsed_time);
    virtual void Act(Real f_elapsed_time);

private:
    std::vector<CRealDeepracerDevice *> m_vecActuators;
    std::vector<CRealDeepracerDevice *> m_vecSensors;
    //Still not sure what we are going to do with these
    std::string strControllerId;
    std::string strARGoSFName;
};

#endif
