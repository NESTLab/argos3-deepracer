#ifndef CCI_DEEPRACER_IMU_SENSOR_H
#define CCI_DEEPRACER_IMU_SENSOR_H

namespace argos {
    class CCI_DeepracerIMUSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos{
    class CCI_DeepracerIMUSensor : public CCI_Sensor {
    public:
        /**
        * Class constructor
        */
        CCI_DeepracerIMUSensor();

        /**
         * Class destructor
         */
        virtual ~CCI_DeepracerIMUSensor() {}

        /**
        * Returns the readings of this sensor
        */
        virtual CVector3 GetAngularVelocities() const = 0;
        virtual CVector3 GetLinearAccelerations() const = 0;
#ifdef ARGOS_WITH_LUA
        virtual void CreateLuaState(lua_State* pt_lua_state) {};

        virtual void ReadingsToLuaState(lua_State* pt_lua_state) {};
#endif
    };
}


#endif