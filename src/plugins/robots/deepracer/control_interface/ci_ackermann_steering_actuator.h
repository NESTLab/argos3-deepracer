#ifndef CCI_ACKERMANN_STEERING_ACTUATOR_H
#define CCI_ACKERMANN_STEERING_ACTUATOR_H

/* To avoid dependency problems when including */
namespace argos {
    class CCI_AckermannSteeringActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>

namespace argos {

    class CCI_AckermannSteeringActuator : public CCI_Actuator {
    public:

        virtual ~CCI_AckermannSteeringActuator() {}

        virtual void SetSteeringAndThrottle(Real f_steering_ang,
                                            Real f_throttle_speed) = 0;

#ifdef ARGOS_WITH_LUA
        virtual void CreateLuaState(lua_State* pt_lua_state){};

        virtual void ReadingsToLuaState(lua_State* pt_lua_state){};
#endif

    protected:

        Real m_fSteeringAngle;

        Real m_fThrottleSpeed;
    };
}

#endif // CCI_ACKERMANN_STEERING_ACTUATOR_H
