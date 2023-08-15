#ifndef REAL_DEEPRACER_ACKERMANN_STEERING_ACTUATOR_H
#define REAL_DEEPRACER_ACKERMANN_STEERING_ACTUATOR_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "real_deepracer_device.h"
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_ackermann_steering_actuator.h>
#include <deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp>

namespace argos {
    class CRealDeepracerAckermannSteeringActuator :
            public CCI_AckermannSteeringActuator,
            public CRealDeepracerDevice {
    public:

        CRealDeepracerAckermannSteeringActuator(const std::shared_ptr<CRealDeepracer>& pt_node);
        virtual ~CRealDeepracerAckermannSteeringActuator();
        virtual void Do(Real f_elapsed_time);
        virtual void SetSteeringAndThrottle(Real f_normalized_steering_ang,
                                            Real f_normalized_throttle);

    private:
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr m_ptServoMsgPublisher;
    };
}

#endif