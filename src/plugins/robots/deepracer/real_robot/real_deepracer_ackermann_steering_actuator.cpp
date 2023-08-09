#include "real_deepracer_ackermann_steering_actuator.h"

CRealDeepracerAckermannSteeringActuator::CRealDeepracerAckermannSteeringActuator(const std::shared_ptr<CRealDeepracer>& pt_node) 
    : CRealDeepracerDevice(*pt_node) {
    
    m_ptServoMsgPublisher = pt_node->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(
        "/ctrl_pkg/servo_msg", 10
    );
}

CRealDeepracerAckermannSteeringActuator::~CRealDeepracerAckermannSteeringActuator() {
}

/****************************************/
/****************************************/

void CRealDeepracerAckermannSteeringActuator::Do(Real f_elapsed_time) {

    // Publish to /ctrl_pkg/servo_msg
    deepracer_interfaces_pkg::msg::ServoCtrlMsg tServoMsg;

    tServoMsg.angle = m_fSteeringAndThrottle[0];
    tServoMsg.throttle = m_fSteeringAndThrottle[1];

    m_ptServoMsgPublisher->publish(tServoMsg);
}

/****************************************/
/****************************************/

void CRealDeepracerAckermannSteeringActuator::SetSteeringAndThrottle(Real f_normalized_steering_ang,
                                                                        Real f_normalized_throttle) {
    m_fSteeringAndThrottle[0] = f_normalized_steering_ang;
    m_fSteeringAndThrottle[1] = f_normalized_throttle;
}



