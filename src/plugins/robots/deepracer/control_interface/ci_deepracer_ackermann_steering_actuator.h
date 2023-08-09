#ifndef ARGOS3_DEEPRACER_CI_DEEPRACER_ACKERMANN_STEERING_ACTUATOR_H
#define ARGOS3_DEEPRACER_CI_DEEPRACER_ACKERMANN_STEERING_ACTUATOR_H

/* To avoid dependency problems when including */
namespace argos {
    class CCI_DeepracerAckermannSteeringActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>

namespace argos {

    class CCI_DeepracerAckermannSteeringActuator : public CCI_Actuator {

    public:
        virtual ~CCI_DeepracerAckermannSteeringActuator() {}

        virtual void SetSteeringAndThrottle(Real f_normalized_steering_ang,
                                            Real f_normalized_throttle) = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state) {};

      virtual void ReadingsToLuaState(lua_State* pt_lua_state) {};
#endif

    protected:

        Real m_fSteeringAndThrottle[2];

    };
}

#endif //ARGOS3_DEEPRACER_CI_DEEPRACER_ACKERMANN_STEERING_ACTUATOR_H
