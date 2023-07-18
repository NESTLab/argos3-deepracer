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
    Destroy();
}

/****************************************/
/****************************************/

void CRealDeepracer::Destroy() {
    //Stop wheels
    //Stop LiDAR spinning (?!??)
    //Reset/clear maps if using gmapping in the future
}

/****************************************/
/****************************************/

#define MAKE_ACTUATOR(CLASSNAME, TAG)					\
   if(str_name == TAG) {						\
      CLASSNAME* pcAct =						\
         new CLASSNAME(GetDSPic());					\
      m_vecActuators.push_back(pcAct);					\
      LOG << "[INFO] Initialized \"" << TAG << "\" actuator " << std::endl; \
      return pcAct;							\
   }

CCI_Actuator* CRealDeepracer::MakeActuator(const std::string& str_name) {
    MAKE_ACTUATOR(CRealDeepracerDifferentialSteeringActuator,
                  "differential_steering");
    return NULL;
}

/****************************************/
/****************************************/

#define MAKE_SENSOR(CLASSNAME, TAG)					\
   if(str_name == TAG) {						\
      CLASSNAME* pcSens =						\
         new CLASSNAME(GetDSPic());					\
      m_vecSensors.push_back(pcSens);					\
      LOG << "[INFO] Initialized \"" << TAG << "\" sensor " << std::endl; \
      return pcSens;							\
   }

CCI_Sensor* CRealDeepracer::MakeSensor(const std::string& str_name) {
    MAKE_SENSOR(CRealDeepracerCameraSensor,
                "camera");
    MAKE_SENSOR(CRealDeepracerEncoderSensor,
                "differential_steering");
    MAKE_SENSOR(CRealDeepracerLIDARSensor,
                "deepracer_lidar");
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