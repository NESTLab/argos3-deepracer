#ifndef DEEPRACER_LIDAR_DEFAULT_SENSOR_H
#define DEEPRACER_LIDAR_DEFAULT_SENSOR_H

#include <map>
#include <string>

namespace argos {
    class CDeepracerLIDARDefaultSensor;
}

#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_lidar_sensor.h>
#include <argos3/plugins/robots/generic/simulator/proximity_default_sensor.h>

namespace argos {

    class CDeepracerLIDARDefaultSensor : public CCI_DeepracerLIDARSensor,
                                         public CSimulatedSensor {
    public:

        CDeepracerLIDARDefaultSensor();

        virtual ~CDeepracerLIDARDefaultSensor();

        virtual void SetRobot(CComposableEntity& c_entity);

        virtual void Init(TConfigurationNode& t_tree);

        virtual void Update();

        virtual void Reset();

        virtual void Destroy();

        virtual Real GetReading(UInt32 un_idx) const;

        virtual size_t GetNumReadings() const;

        virtual void PowerOn();

        virtual void PowerOff();

    private:

        /** Readings of the LIDAR sensor */
        Real* m_pfReadings;

        /** Number of readings of the LIDAR sensor */
        size_t m_unNumReadings;

        /** Reference to embodied entity associated to this sensor */
        CEmbodiedEntity* m_pcEmbodiedEntity;

        /** Reference to proximity sensor equipped entity associated to this sensor */
        CProximitySensorEquippedEntity* m_pcProximityEntity;

        /** Reference to controllable entity associated to this sensor */
        CControllableEntity* m_pcControllableEntity;

        /** Flag to show rays in the simulator */
        bool m_bShowRays;

        /** Flag to indicate whether the LIDAR is powered on or off */
        bool m_bPowerStateOn;

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
