#ifndef REAL_DEEPRACER_DEVICE_H
#define REAL_DEEPRACER_DEVICE_H
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>

using namespace argos;

class CRealDeepracer; // forward declare CRealDeepracer

class CRealDeepracerDevice {
public:

    CRealDeepracerDevice(const std::shared_ptr<rclcpp::Node>& pt_node_handle) : m_ptNodeHandle(pt_node_handle) {}
    virtual ~CRealDeepracerDevice() {}
    virtual void Do(Real f_elapsed_time) {}

protected:

    std::shared_ptr<rclcpp::Node> m_ptNodeHandle;
};

#endif // REAL_DEEPRACER_DEVICE_H
