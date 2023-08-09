#ifndef CCI_DEEPRACER_GYROSCOPE_SENSOR_H
#define CCI_DEEPRACER_GYROSCOPE_SENSOR_H

namespace argos {
    class CCI_DeepracerIMUSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>

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
        virtual std::vector<Real> GetOrientations() const = 0;
        virtual std::vector<Real> GetAngularVelocities() const = 0;
        virtual std::vector<Real> GetLinearAccelerations() const = 0;

    };
}


#endif