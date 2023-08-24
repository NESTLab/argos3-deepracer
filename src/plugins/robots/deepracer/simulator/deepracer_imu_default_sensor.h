#ifndef DEEPRACER_IMU_DEFAULT_SENSOR_H
#define DEEPRACER_IMU_DEFAULT_SENSOR_H

#include <map>
#include <string>

namespace argos {
    class CDeepracerIMUDefaultSensor;
    class CEmbodiedEntity;
}

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_imu_sensor.h>

namespace argos {

    class CDeepracerIMUDefaultSensor : public CSimulatedSensor,
                                       public CCI_DeepracerIMUSensor {
    public:

        struct SEulerAnglesInCRadians {
            CRadians X;
            CRadians Y;
            CRadians Z;

            inline void ToCVector3(CVector3& c_vector) {
                c_vector.SetX(X.GetValue());
                c_vector.SetY(Y.GetValue());
                c_vector.SetZ(Z.GetValue());
            }
        };

    public:

        CDeepracerIMUDefaultSensor();

        virtual ~CDeepracerIMUDefaultSensor() {}

        virtual void SetRobot(CComposableEntity& c_entity);

        virtual void Init(TConfigurationNode& t_tree);

        virtual void Update();

        virtual void Reset();

    protected:

        /** Reference to embodied entity associated to this sensor */
        CEmbodiedEntity* m_pcEmbodiedEntity;

        /** Random number generator */
        CRandom::CRNG* m_pcRNG;

        /** Whether to add noise or not */
        bool m_bAddNoise;

        /** Noise range on linear accelerations */
        CRange<Real> m_cLinAccNoiseRange;

        /** Noise range on angular velocities */
        CRange<CRadians> m_cAngVelNoiseRange;

        /** Reference to the space */
        CSpace& m_cSpace;

        /** Current recorded linear position */
        CVector3 m_cCurrentPosition;

        /** Current recorded orientation */
        CQuaternion m_cCurrentOrientation;

        /** Current linear velocity */
        CVector3 m_cCurrentLinVel;

        /** Struct to store and convert computed angular velocities */
        SEulerAnglesInCRadians m_sAngVelEuler;

        /** Number of ticks in one second */
        Real m_fNumTicksPerSec;

        /** Previous simulation time */
        Real m_fPreviousTime;

        /** Current simulation time */
        Real m_fCurrentTime;

        /** Simulation time difference between current and previous */
        Real m_fDeltaTime;
    };

}

#endif