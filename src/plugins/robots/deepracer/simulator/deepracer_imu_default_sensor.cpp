#include "deepracer_imu_default_sensor.h"

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

    /****************************************/
    /****************************************/

    CDeepracerIMUDefaultSensor::CDeepracerIMUDefaultSensor() : m_pcEmbodiedEntity(nullptr),
                                                               m_pcRNG(nullptr),
                                                               m_bAddNoise(false),
                                                               m_cCurrentPosition(CVector3::ZERO),
                                                               m_cCurrentOrientation(CQuaternion()),
                                                               m_cCurrentLinVel(CVector3::ZERO),
                                                               m_cSpace(CSimulator::GetInstance().GetSpace()) {}

    /****************************************/
    /****************************************/

    void CDeepracerIMUDefaultSensor::SetRobot(CComposableEntity& c_entity) {
        m_pcEmbodiedEntity    = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
        m_cCurrentPosition    = m_pcEmbodiedEntity->GetOriginAnchor().Position;
        m_cCurrentOrientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
    }

    /****************************************/
    /****************************************/

    void CDeepracerIMUDefaultSensor::Init(TConfigurationNode& t_tree) {
        try {
            CCI_PositioningSensor::Init(t_tree);
            /* Parse noise range */
            GetNodeAttributeOrDefault(t_tree, "lin_acc_noise_range", m_cLinAccNoiseRange, m_cLinAccNoiseRange);
            GetNodeAttributeOrDefault(t_tree, "ang_vel_noise_range", m_cAngVelNoiseRange, m_cAngVelNoiseRange);
            if (m_cLinAccNoiseRange.GetSpan() != 0 ||
                m_cAngVelNoiseRange.GetSpan() != CRadians::ZERO) {
                m_bAddNoise = true;
                m_pcRNG     = CRandom::CreateRNG("argos");
            }

            /* Populate the number of ticks in one second */
            UInt32 nNumTicksPerSec;

            TConfigurationNode& tNode =
                GetNode(GetNode(GetSimulator().GetConfigurationRoot(), "framework"), "experiment");
            GetNodeAttribute(tNode, "ticks_per_second", nNumTicksPerSec);

            m_fNumTicksPerSec = static_cast<Real>(nNumTicksPerSec);

            /* sensor is enabled by default */
            Enable();
        } catch (CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Initialization error in default IMU sensor", ex);
        }
    }

    /****************************************/
    /****************************************/

    void CDeepracerIMUDefaultSensor::Update() {
        /* sensor is disabled--nothing to do */
        if (IsDisabled()) {
            return;
        }

        /* Get current sim time */
        m_fCurrentTime  = static_cast<Real>(m_cSpace.GetSimulationClock()) / m_fNumTicksPerSec;
        m_fDeltaTime    = m_fCurrentTime - m_fPreviousTime;
        m_fPreviousTime = m_fCurrentTime;

        /* Compute linear acceleration */
        m_sReading.LinAcceleration = m_cCurrentLinVel; // act as a temporary storage

        m_cCurrentLinVel = (m_pcEmbodiedEntity->GetOriginAnchor().Position - m_cCurrentPosition) / m_fDeltaTime;

        m_cCurrentPosition = m_pcEmbodiedEntity->GetOriginAnchor().Position;

        /* Compute angular velocity */
        (m_pcEmbodiedEntity->GetOriginAnchor().Orientation - m_cCurrentOrientation).ToEulerAngles(m_sAngVelEuler.X, m_sAngVelEuler.Y, m_sAngVelEuler.Z);

        m_sAngVelEuler.ToCVector3(m_sReading.AngVelocity);

        m_sReading.AngVelocity /= m_fDeltaTime;

        m_cCurrentOrientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;

        /* Add noise */
        if (m_bAddNoise) {
            m_sReading.LinAcceleration += CVector3(m_pcRNG->Uniform(m_cLinAccNoiseRange),
                                                   m_pcRNG->Uniform(m_cLinAccNoiseRange),
                                                   m_pcRNG->Uniform(m_cLinAccNoiseRange));
            m_sReading.AngVelocity += CVector3(m_pcRNG->Uniform(m_cAngVelNoiseRange),
                                               m_pcRNG->Uniform(m_cAngVelNoiseRange),
                                               m_pcRNG->Uniform(m_cAngVelNoiseRange));
        }
    }

    /****************************************/
    /****************************************/

    void CDeepracerIMUDefaultSensor::Reset() {
        m_cCurrentPosition    = m_pcEmbodiedEntity->GetOriginAnchor().Position;
        m_cCurrentOrientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
    }

    /****************************************/
    /****************************************/

    REGISTER_SENSOR(CDeepracerIMUDefaultSensor,
                    "deepracer_imu", "default",
                    "Carlo Pinciroli [ilpincy@gmail.com], Khai Yi Chin [khaiyichin@gmail.com]",
                    "1.0",
                    "A AWS Deepracer IMU sensor.",

                    "This sensor returns the current angular velocity and linear acceleration of\n"
                    "a robot. This sensor can be used with any robot, since it accesses only the\n"
                    "body component. In controllers, you must include the ci_deepracer_imu_sensor.h header.\n\n"

                    "This sensor is enabled by default.\n\n"

                    "REQUIRED XML CONFIGURATION\n\n"
                    "  <controllers>\n"
                    "    ...\n"
                    "    <my_controller ...>\n"
                    "      ...\n"
                    "      <sensors>\n"
                    "        ...\n"
                    "        <positioning implementation=\"default\" />\n"
                    "        ...\n"
                    "      </sensors>\n"
                    "      ...\n"
                    "    </my_controller>\n"
                    "    ...\n"
                    "  </controllers>\n\n"

                    "OPTIONAL XML CONFIGURATION\n\n"

                    "It is possible to add uniform noise to the sensor, thus matching the\n"
                    "characteristics of a real robot better. You can add noise through the\n"
                    "attributes 'lin_acc_noise_range', and 'ang_vel_noise_range'.\n"
                    "Attribute 'lin_acc_noise_range' regulates the noise range on the linear acceleration returned\n"
                    "by the sensor. Attribute 'ang_vel_noise_range' sets the noise range on the angular velocity\n"
                    "(values expressed in rad/s).\n\n"

                    "  <controllers>\n"
                    "    ...\n"
                    "    <my_controller ...>\n"
                    "      ...\n"
                    "      <sensors>\n"
                    "        ...\n"
                    "        <positioning implementation=\"default\"\n"
                    "                     lin_acc_noise_range=\"-0.1:0.2\"\n"
                    "                     ang_vel_noise_range=\"-10.5:13.7\"\n"
                    "        ...\n"
                    "      </sensors>\n"
                    "      ...\n"
                    "    </my_controller>\n"
                    "    ...\n"
                    "  </controllers>\n\n"

                    "OPTIONAL XML CONFIGURATION\n\n"

                    "None.\n",

                    "Usable");

}