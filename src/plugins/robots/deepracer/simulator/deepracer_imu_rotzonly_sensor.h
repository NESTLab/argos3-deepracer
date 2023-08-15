#ifndef DEEPRACER_IMU_ROTZONLY_SENSOR_H
#define DEEPRACER_IMU_ROTZONLY_SENSOR_H

#include <string>
#include <map>

namespace argos {
    class CDeepracerIMURotZOnlySensor;
    class CDeepracerIMUSensorEquippedEntity;
}

#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_imu_sensor.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

    class CDeepracerIMURotZOnlySensor : public CSimulatedSensor,
                                        public CCI_DeepracerIMUSensor {

    public:

        CDeepracerIMURotZOnlySensor();

        virtual ~CDeepracerIMURotZOnlySensor() {}

        virtual void SetRobot(CComposableEntity& c_entity);

        virtual void Init(TConfigurationNode& t_tree);

        virtual void Update();

        virtual void Reset();

    protected:

        /** Reference to embodied entity associated to this sensor */
        CEmbodiedEntity* m_pcEmbodiedEntity;

        /** Reference to light sensor equipped entity associated to this sensor */
        // CLightSensorEquippedEntity* m_pcLightEntity;

        /** Reference to IMU sensor equipped entity associated to this sensor */
        CDeepracerIMUSensorEquippedEntity* m_pcIMUEquippedEntity;

        /** Reference to controllable entity associated to this sensor */
        CControllableEntity* m_pcControllableEntity;

        /** Flag to show rays in the simulator */
        // bool m_bShowRays;

        /** Random number generator */
        CRandom::CRNG* m_pcRNG;

        /** Whether to add noise or not */
        bool m_bAddNoise;

        /** Noise range */
        CRange<Real> m_cNoiseRange;

        /** Reference to the space */
        CSpace& m_cSpace;
    };

}

#endif