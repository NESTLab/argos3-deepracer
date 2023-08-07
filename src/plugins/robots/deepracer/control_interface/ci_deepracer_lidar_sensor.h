//
// Created by OWNER on 7/12/2023.
//

#ifndef CI_DEEPRACER_LIDAR_SENSOR_H
#define CI_DEEPRACER_LIDAR_SENSOR_H

namespace argos {
    class CCI_DeepracerLIDARSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>

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
        virtual long GetReading(UInt32 un_idx) const = 0;

        /**
         * Returns the readings of this sensor
         */
        virtual size_t GetNumReadings() const = 0;
        /*
         * Switches the sensor power on.
        */
        virtual void PowerOn() = 0;

        /*
         * Switches the sensor power off.
         */
        virtual void PowerOff() = 0;

        /*
         * Switches the laser on.
         */
        virtual void LaserOn() = 0;

        /*
         * Switches the laser off.
         */
        virtual void LaserOff() = 0;
    };
}

#endif //ARGOS3_DEEPRACER_CI_DEEPRACER_LIDAR_SENSOR_H
