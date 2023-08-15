#ifndef ACKERMANN_STEERING_ACTUATOR_DEFAULT_H
#define ACKERMANN_STEERING_ACTUATOR_DEFAULT_H

#include <map>
#include <string>

namespace argos {
    class CAckermannSteeringDefaultActuator;
}

#include <argos3/core/simulator/actuator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_ackermann_steering_actuator.h>
#include "ackermann_wheeled_entity.h"

namespace argos {

    class CAckermannSteeringDefaultActuator : public CSimulatedActuator,
                                              public CCI_AckermannSteeringActuator {
    public:

        enum ACKERMANN_STEERING {
            REAR_LEFT_WHEEL   = 0,
            REAR_RIGHT_WHEEL  = 1,
            FRONT_LEFT_WHEEL  = 2,
            FRONT_RIGHT_WHEEL = 3
        };

    public:

        /**
         * @brief Constructor.
         */
        CAckermannSteeringDefaultActuator();

        /**
         * @brief Destructor.
         */
        virtual ~CAckermannSteeringDefaultActuator() {}

        virtual void SetRobot(CComposableEntity& c_entity);

        virtual void Init(TConfigurationNode& t_tree);

        /**
         * @brief Set the steering and throttle speed of the robot.
         *
         * @param f_steering_ang Desired steering angle in radians.
         * @param f_throttle_speed Desired throttle velocity in cm/s.
         */
        virtual void SetSteeringAndThrottle(Real f_steering_ang,
                                            Real f_throttle_speed);

        virtual void Update();

        virtual void Reset();

    protected:

        CAckermannWheeledEntity* m_pcAckermannWheeledEntity;

        /** Random number generator */
        CRandom::CRNG* m_pcRNG;

        /** Noise bias for each wheel */
        Real m_fNoiseBias[4];

        /** Noise factor average (Gaussian model) for each wheel  */
        Real m_fNoiseFactorAvg[4];

        /** Noise factor stddev (Gaussian model) for each wheel  */
        Real m_fNoiseFactorStdDev[4];
    };

}

#endif