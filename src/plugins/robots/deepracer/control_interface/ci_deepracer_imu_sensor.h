#ifndef CCI_DEEPRACER_IMU_SENSOR_H
#define CCI_DEEPRACER_IMU_SENSOR_H

namespace argos {
    class CCI_DeepracerIMUSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {
    class CCI_DeepracerIMUSensor : public CCI_Sensor {
    public:

        struct SReading {
            CVector3 AngVelocity;
            CVector3 LinAcceleration;
        };

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
         * Returns the reading of this sensor
         */
        inline const SReading& GetReading() const {
            return m_sReading;
        }

#ifdef ARGOS_WITH_LUA
        virtual void CreateLuaState(lua_State* pt_lua_state) {}

        virtual void ReadingsToLuaState(lua_State* pt_lua_state) {}
#endif

    protected:

        SReading m_sReading;
        CRadians m_cAngle;
        CVector3 m_cAxis;
    };
}

#endif