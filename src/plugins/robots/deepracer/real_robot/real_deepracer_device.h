#ifndef REAL_DEEPRACER_DEVICE_H
#define REAL_DEEPRACER_DEVICE_H
#include <signal.h>
#include <memory> //Not sure but I just include this for now
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer.h>

using namespace argos;

class CRealDeepracer; // forward declare CRealDeepracer

class CRealDeepracerDevice {
public:
    CRealDeepracerDevice(CRealDeepracer& t_node_handle) : m_tNodeHandle(t_node_handle) {}
    virtual ~CRealDeepracerDevice() {}
    virtual void Do(Real f_elapsed_time) = 0;

protected:
    CRealDeepracer& m_tNodeHandle;

};

#endif //REAL_DEEPRACER_DEVICE_H
