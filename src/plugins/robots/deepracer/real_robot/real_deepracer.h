#ifndef REAL_DEEPRACER_H
#define REAL_DEEPRACER_H

#include "real_deepracer_device.h"
#include "real_deepracer_differential_steering_actuator.h"
#include "real_deepracer_camera_sensor.h"
#include "real_deepracer_lidar_sensor.h"
#include <argos3/core/real_robot/real_robot.h>
#include <argos3/core/utility/rate.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>


using namespace argos;

class CRealDeepracer : public CRealRobot {
public:

    CRealDeepracer();
    virtual ~CRealDeepracer();
    virtual void InitRobot();
    virtual void Destroy();
    virtual CCI_Actuator* MakeActuator(const std::string& str_name);
    virtual CCI_Sensor* MakeSensor(const std::string& str_name);
    virtual void Sense(Real f_elapsed_time);
    virtual void Act(Real f_elapsed_time);


private:
    enum string_code {
        differentialSteering,
        camera,
        lidar,
        encoder
    };
    virtual string_code hashit (std::string &const inString);

    std::vector<CRealDeepracerDevice*> m_vecActuators;
    std::vector<CRealDeepracerDevice*> m_vecSensors;
    class NodeHandler : public rclcpp::Node{
    public: // init
        NodeHandler(): Node("node_handler")
        {   //TODO: check the deepracer's code to match all the ros topics
            //All the publisher(s)
            publisher_differentialsteering = this->create_publisher<std_msgs::msg::String>("topic", 10);
            //All the subscriber(s)
            subscription_encoders = this->create_subscription<std_msgs::msg::String>(
                    "encoder_topic", 10, std::bind(&NodeHandler::topic_callback_subscription_encoder, this, _1));
            subscription_camera = this->create_subscription<std_msgs::msg::String>(
                    "camera", 10, std::bind(&NodeHandler::topic_callback_subscription_camera, this, _1));
            subscription_lidar = this->create_subscription<std_msgs::msg::String>(
                    "lidar", 10, std::bind(&NodeHandler::topic_callback_subscription_lidar, this, _1));

        }

    private: // callback < each subscriber has a callback <?!?>
        //TODO: check this and change the msg type to the correct type
        void topic_callback_subscription_encoder(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s' from encoder", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_encoders;

        void topic_callback_subscription_camera(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s' from camera", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_camera;

        void topic_callback_subscription_lidar(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s' from lidar", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_lidar;
    };
};


#endif
