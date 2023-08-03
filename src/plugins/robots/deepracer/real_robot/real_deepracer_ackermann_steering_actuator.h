#ifndef REAL_DEEPRACER_DIFFERENTIAL_STEERING_ACTUATOR_H
#define REAL_DEEPRACER_DIFFERENTIAL_STEERING_ACTUATOR_H

#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "real_deepracer_device.h"

class CRealDeepracerAckermannSteeringActuator :
        public CCI_DifferentialSteeringActuator,
        public CRealDeepracerDevice{
public:

    CRealDeepracerAckermannSteeringActuator(rclcpp::Node& t_node_handler);
    virtual ~CRealDeepracerAckermannSteeringActuator();
    virtual void Do(Real f_elapsed_time);
    virtual void SetLinearVelocity(Real f_left_velocity,
                                   Real f_right_velocity);
};

#endif
