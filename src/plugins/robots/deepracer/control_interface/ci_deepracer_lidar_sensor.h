#ifndef CCI_DEEPRACER_LIDAR_SENSOR_H
#define CCI_DEEPRACER_LIDAR_SENSOR_H

namespace argos {
    class CCI_DeepracerLIDARSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {
    class CCI_DeepracerLIDARSensor : public CCI_Sensor {
    public:

        /**
         * Class constructor
         */
        CCI_DeepracerLIDARSensor();

        /**
         * Class destructor
         */
        virtual ~CCI_DeepracerLIDARSensor() {}

        /**
         * Returns the readings of this sensor
         */
        virtual CVector3 GetReading() const = 0;

#ifdef ARGOS_WITH_LUA
        virtual void CreateLuaState(lua_State* pt_lua_state){};

        virtual void ReadingsToLuaState(lua_State* pt_lua_state){};
#endif
    };
}

#endif // CCI_DEEPRACER_LIDAR_SENSOR_H
