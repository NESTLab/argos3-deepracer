#include "real_deepracer.h"

/****************************************/
/****************************************/

CRealDeepracer::CRealDeepracer(){
}
/****************************************/
/****************************************/
CRealDeepracer::~CRealDeepracer(){
}
/****************************************/
/****************************************/

void CRealDeepracer::InitRobot(){
    // Initialize DeepRacer Robot => Might deal with the IP Addresses of the robot
    // Throw communication errors if it has any
    // Make sure the robot starts from a clean state
    rclcpp::init(0, 0); // argc, argv but I just set them to 0 here
    rclcpp::spin(std::make_shared<NodeHandler>());

    // Init ros2
    // Publish to cmd_vel to control the actuator
    // Subscribe to the camera, lidar and cmd_vel to "sense" the encoders and updates the values
    Destroy(); // This will close the topics, do we want to destroy it ?
}

/****************************************/
/****************************************/

void CRealDeepracer::Destroy() {
    //Stop wheels
    //Stop LiDAR spinning (?!??)
    //Reset/clear maps if using gmapping in the future
    rclcpp::shutdown(); // Shut down ros2 service
}

/****************************************/
/****************************************/
//This is to deal with the string switch cases in c++
//https://stackoverflow.com/questions/650162/why-cant-the-switch-statement-be-applied-to-strings


CRealDeepracer::string_code CRealDeepracer::hashit(std::string &const inString) {
    if (inString == "differential_steering") return differentialSteering;
    if (inString == "camera") return camera;
    if (inString == "deepracer_lidar") return lidar;
    if (inString == "encoder") return encoder;
}

CCI_Actuator* CRealDeepracer::MakeActuator(const std::string& str_name) {
    //Determines that the deepracer has that actuator or not
    switch(hashit(str_name)){
        case differentialSteering: {
            CRealDeepracerDifferentialSteeringActuator *pcAct =
                    new CRealDeepracerDifferentialSteeringActuator(*NodeHandler.publisher_differentialsteering );
            m_vecActuators.push_back(pcAct);
            LOG << "[INFO] Initialized \"" << "differential_steering" << "\" actuator " << std::endl;
            break;
        }

        default:
            break;

    }
    return NULL;
}

/****************************************/
/****************************************/


CCI_Sensor* CRealDeepracer::MakeSensor(const std::string& str_name) {
    //Determines that the deepracer has that actuator or not
    //TODO: Fix coding syntax and buggies
    switch(hashit(str_name)){
        case lidar: {
            CRealDeepracerLidarSensor *pcSensLi = new CRealDeepracerLidarSensor(*NodeHandler.subscription_lidar);
            m_vecSensors.push_back(pcSensLi);
            LOG << "[INFO] Initialized \"" << "deepracer lidar" << "\" sensor " << std::endl;
            break;
        }

        case camera: {
            CRealDeepracerCameraSensor *pcSensCam = new CRealDeepracerCameraSensor(*NodeHandler.subsubscription_camera););
            m_vecSensors.push_back(pcSensCam);
            LOG << "[INFO] Initialized \"" << "camera" << "\" sensor " << std::endl;
            break;
        }
        default:
            break;
    }
    return NULL;
}

/****************************************/
/****************************************/

void CRealDeepracer::Sense(Real f_elapsed_time) {
    for(size_t i = 0; i < m_vecSensors.size(); ++i) {
        m_vecSensors[i]->Do(f_elapsed_time);
    }
}

/****************************************/
/****************************************/

void CRealDeepracer::Act(Real f_elapsed_time) {
    for(size_t i = 0; i < m_vecActuators.size(); ++i) {
        m_vecActuators[i]->Do(f_elapsed_time);
    }
}

/****************************************/
/****************************************/