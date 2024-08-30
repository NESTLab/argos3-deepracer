#ifndef DEEPRACER_DRIVE_DEMO_H
#define DEEPRACER_DRIVE_DEMO_H

/*
 * Include some necessary headers.
 */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_ackermann_steering_actuator.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_imu_sensor.h>
#include <argos3/plugins/robots/deepracer/control_interface/ci_deepracer_lidar_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CDeepracerDriveDemo : public CCI_Controller {
public:

    /* Class constructor. */
    CDeepracerDriveDemo();

    /* Class destructor. */
    virtual ~CDeepracerDriveDemo() {}

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><deepracer_diffusion_controller> section.
     */
    virtual void Init(TConfigurationNode& t_node);

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    virtual void ControlStep();

    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     * In this example controller there is no need for resetting anything,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Reset() {}

    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy() {}

private:

    /* Pointer to the Ackermann steering actuator */
    CCI_AckermannSteeringActuator* m_pcWheels;
    /* Pointer to the AWS DeepRacer LIDAR sensor */
    CCI_DeepracerLIDARSensor* m_pcLIDAR;
    /* Pointer to the AWS DeepRacer IMU sensor */
    CCI_DeepracerIMUSensor* m_pcIMU;

    /*
     * The following variables are used as parameters for the
     * algorithm. You can set their value in the <parameters> section
     * of the XML configuration file, under the
     * <controllers><footbot_diffusion_controller> section.
     */

    /* Time to switch steering */
    UInt32 m_unSwitchingTime;
    /* Wheel speed */
    Real m_fWheelVelocity;
    /* Current steering angle */
    Real m_fCurrentSteeringAngle = M_2_PI;
    /* Time step counter */
    UInt32 m_unCounter = 0;
};

#endif