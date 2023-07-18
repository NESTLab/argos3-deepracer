#ifndef REAL_DEEPRACER_H
#define REAL_DEEPRACER_H

#include "real_deepracer_device.h"


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

    std::vector<CRealDeepracerDevice*> m_vecActuators;
    std::vector<CRealDeepracerDevice*> m_vecSensors;
};


#endif
