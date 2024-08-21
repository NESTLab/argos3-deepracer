#include "real_deepracer.h"
#include "real_deepracer_device.h"
#include "real_deepracer_imu_sensor.h"
#include "real_deepracer_ackermann_steering_actuator.h"
// #include "real_deepracer_camera_sensor.h"
#include "real_deepracer_lidar_sensor.h"

/****************************************/
/****************************************/

CRealDeepracer::CRealDeepracer() : rclcpp::Node("deepracer_node") {
}

/****************************************/
/****************************************/

void CRealDeepracer::Destroy() {
    //Reset/clear maps if using gmapping in the future
}

/****************************************/
/****************************************/

#define MAKE_SENSOR(CLASSNAME, TAG)                                                      \
    if (str_name == TAG) {                                                               \
        CLASSNAME *pcSens =                                                              \
            new CLASSNAME(                                                               \
                GetNodeHandlePtr()                                                       \
            );                                                                           \
        m_vecSensors.push_back(pcSens);                                                  \
        LOG << "[INFO] Initialized \"" << TAG << "\" sensor " << std::endl;              \
        return pcSens;                                                                   \
    }

CCI_Sensor* CRealDeepracer::MakeSensor(const std::string& str_name) {
//    MAKE_SENSOR(CRealDeepracerCameraSensor,
//                "camera");
    MAKE_SENSOR(CRealDeepracerIMUSensor,
                "deepracer_imu");
    MAKE_SENSOR(CRealDeepracerLIDARSensor,
                "deepracer_lidar");
    return NULL;
}

/****************************************/
/****************************************/

#define MAKE_ACTUATOR(CLASSNAME, TAG)                                                    \
    if (str_name == TAG) {                                                               \
        CLASSNAME *pcAct =                                                               \
            new CLASSNAME(                                                               \
                GetNodeHandlePtr()                                                       \
            );                                                                           \
        m_vecActuators.push_back(pcAct);                                                 \
        LOG << "[INFO] Initialized \"" << TAG << "\" actuator " << std::endl;            \
        return pcAct;                                                                    \
    }

CCI_Actuator* CRealDeepracer::MakeActuator(const std::string& str_name) {
    MAKE_ACTUATOR(CRealDeepracerAckermannSteeringActuator,
                  "ackermann_steering");
    return NULL;
}

/****************************************/
/****************************************/

void CRealDeepracer::Sense(Real f_elapsed_time) {
    /* Tell ROS to collect messages */
    rclcpp::spin_some(GetNodeHandlePtr());
}

/****************************************/
/****************************************/

void CRealDeepracer::Act(Real f_elapsed_time) {

    // Go through actuators and let them do their thing
    for(size_t i = 0; i < m_vecActuators.size(); ++i) {
        m_vecActuators[i]->Do(f_elapsed_time);
    }

    // Tell ROS to publish the messages
    rclcpp::spin_some(GetNodeHandlePtr());
}
