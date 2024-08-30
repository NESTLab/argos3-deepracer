/* Include the controller definition */
#include "deepracer_hello_world.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CDeepracerHelloWorld::CDeepracerHelloWorld() : m_pcLIDAR(NULL),
                                               m_pcIMU(NULL) {}

/****************************************/
/****************************************/

void CDeepracerHelloWorld::Init(TConfigurationNode& t_node) {
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
     * file at the <controllers><deepracer_diffusion><actuators> and
     * <controllers><deepracer_diffusion><sensors> sections. If you forgot to
     * list a device in the XML and then you request it here, an error
     * occurs.
     */
    m_pcLIDAR = GetSensor<CCI_DeepracerLIDARSensor>("deepracer_lidar");
    m_pcIMU = GetSensor<CCI_DeepracerIMUSensor>("deepracer_imu");
}

/****************************************/
/****************************************/

void CDeepracerHelloWorld::ControlStep() {

    if (++m_unCounter % 10 == 0) {
        // Print IMU readings
        auto sReading = m_pcIMU->GetReading();

        LOG << "IMU Angular Velocity (x, y, z) = (" << sReading.AngVelocity.GetX()
            << ", " << sReading.AngVelocity.GetY()
            << ", " << sReading.AngVelocity.GetZ()
            << ")" << std::endl;

        LOG << "IMU Linear Acceleration (x, y, z) = (" << sReading.LinAcceleration.GetX()
            << ", " << sReading.LinAcceleration.GetY()
            << ", " << sReading.LinAcceleration.GetZ()
            << ")" << std::endl;

        // Print LIDAR messages
        LOG << "LIDAR ranges (" << "points) = [" << std::endl;
        for (size_t i = 0; i < m_pcLIDAR->GetNumReadings(); ++i) {
            LOG << m_pcLIDAR->GetReading(i) << " ";
        }
        LOG << std::endl << "]" << std::endl << std::endl;

        LOG.Flush(); // temporary fix to ensure log outputs get out

        m_unCounter = 0; // reset counter
    }
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
REGISTER_CONTROLLER(CDeepracerHelloWorld, "deepracer_hello_world_controller")