/* Include the controller definition */
#include "deepracer_drive_demo.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CDeepracerDriveDemo::CDeepracerDriveDemo() : m_pcWheels(NULL),
                                             m_pcLIDAR(NULL),
                                             m_pcIMU(NULL),
                                             m_unSwitchingTime(700),
                                             m_fWheelVelocity(2.0f) {}

/****************************************/
/****************************************/

void CDeepracerDriveDemo::Init(TConfigurationNode &t_node) {
    /*
     * Get sensor/actuator handles
     *
     * The passed string (ex. "ackermann_steering") corresponds to the
     * XML tag of the device whose handle we want to have. For a list of
     * allowed values, type at the command prompt:
     *
     * $ argos3 -q actuators
     *
     * to have a list of all the possible actuators, or
     *
     * $ argos3 -q sensors
     *
     * to have a list of all the possible sensors.
     *
     * NOTE: ARGoS creates and initializes actuators and sensors
     * internally, on the basis of the lists provided the configuration
     * file at the <controllers><deepracer_drive_demo><actuators> and
     * <controllers><deepracer_drive_demo><sensors> sections. If you forgot to
     * list a device in the XML and then you request it here, an error
     * occurs.
     */
    m_pcWheels = GetActuator<CCI_AckermannSteeringActuator>("ackermann_steering");
    GetNodeAttributeOrDefault(t_node, "switching_time", m_unSwitchingTime, m_unSwitchingTime);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CDeepracerDriveDemo::ControlStep() {
    // Flip steering direction
    if (++m_unCounter % m_unSwitchingTime == 0) {
        m_fCurrentSteeringAngle = -m_fCurrentSteeringAngle;
        m_unCounter             = 0;
    }

    m_pcWheels->SetSteeringAndThrottle(m_fCurrentSteeringAngle, m_fWheelVelocity);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CDeepracerDriveDemo, "deepracer_drive_demo_controller")