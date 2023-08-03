#include "real_deepracer_differential_steering_actuator.h"

CRealDeepracerDifferentialSteeringActuator::CRealDeepracerDifferentialSteeringActuator(rclcpp::Node& t_node_handler) {
    // Create the publisher
    std::shared_ptr<PublisherT> publisher_ = t_node_handler->create_publisher<std_msgs::msg::String>("/cmd_vel", 10);
}

CRealDeepracerDifferentialSteeringActuator::~CRealDeepracerDifferentialSteeringActuator() {
}

/****************************************/
/****************************************/
void CRealDeepracerDifferentialSteeringActuator::Do(int f_elapsed_time) {

}
void CRealDeepracerDifferentialSteeringActuator::SetLinearVelocity(int f_left_velocity, int f_right_velocity) {
    // Publishing to the topic
    auto message = std_msgs::msg::geometry_msgs();
    message.linear.x = (f_left_velocity + f_right_velocity)/2; // Might need kinematics here
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    m_tNodeHandle.publisher_->publish(message);
}



