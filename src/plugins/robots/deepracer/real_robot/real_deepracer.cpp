#include "real_deepracer.h"
#include "real_deepracer_device.h"
// #include "real_deepracer_ackermann_steering_actuator.h"
// #include "real_deepracer_encoder_sensor.h"
// #include "real_deepracer_camera_sensor.h"
#include "real_deepracer_lidar_sensor.h"

/****************************************/
/****************************************/

CRealDeepracer::CRealDeepracer() : rclcpp::Node("deepracer_node") {
}

/****************************************/
/****************************************/

void CRealDeepracer::InitRobot(std::string str_argos_fname, std::string str_controller_id) {
    //Update the name and id
    this->strARGoSFName = str_argos_fname;
    this->strControllerId = str_controller_id;

    // Throw communication errors if it has any
    // Make sure the robot starts from a clean state
    Destroy(); // This will close the topics, do we want to destroy it ?
}

/****************************************/
/****************************************/

void CRealDeepracer::Destroy() {
    //Stop wheels
    //Reset/clear maps if using gmapping in the future
//    rclcpp::shutdown(); // Shut down ros2 service TODO: should this be here?
}

/****************************************/
/****************************************/

#define MAKE_SENSOR(CLASSNAME, TAG)					\
   if(str_name == TAG) {						\
      CLASSNAME* pcSens =						\
         new CLASSNAME(this->std::enable_shared_from_this<CRealDeepracer>::shared_from_this());					\
      m_vecSensors.push_back(pcSens);					\
      LOG << "[INFO] Initialized \"" << TAG << "\" sensor " << std::endl; \
      return pcSens;							\
   }

CCI_Sensor* CRealDeepracer::MakeSensor(const std::string& str_name) {
//    MAKE_SENSOR(CRealDeepracerCameraSensor,
//                "camera");
//    MAKE_SENSOR(CRealDeepracerEncoderSensor,
//                "ackermann_steering");
    MAKE_SENSOR(CRealDeepracerLIDARSensor,
                "lidar");
    return NULL;
}


/****************************************/
/****************************************/

// #define MAKE_ACTUATOR(CLASSNAME, TAG)					\
//    if(str_name == TAG) {						\
//       CLASSNAME* pcAct =						\
//          new CLASSNAME(this->node_handler);					\
//       m_vecActuators.push_back(pcAct);					\
//       LOG << "[INFO] Initialized \"" << TAG << "\" actuator " << std::endl; \
//       return pcAct;							\
//    }

// CCI_Actuator* CRealDeepracer::MakeActuator(const std::string& str_name) {
// //    MAKE_ACTUATOR(CRealDeepracerAckermannSteeringActuator, // TODO: name change
// //                  "ackermann_steering");
//     return NULL;
// }
/****************************************/
/****************************************/

void CRealDeepracer::Sense(Real f_elapsed_time) {
    for(size_t i = 0; i < m_vecSensors.size(); ++i) {
        m_vecSensors[i]->Do(f_elapsed_time);
    }
}

void CRealDeepracer::Act(Real f_elapsed_time) {
    for(size_t i = 0; i < m_vecActuators.size(); ++i) {
        m_vecActuators[i]->Do(f_elapsed_time);
    }
}
