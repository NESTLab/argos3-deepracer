#include "ackermann_steering_default_actuator.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/plugins/factory.h>

namespace argos {

    /****************************************/
    /****************************************/

    CAckermannSteeringDefaultActuator::CAckermannSteeringDefaultActuator()
        : m_pcAckermannWheeledEntity(nullptr),
          m_pcRNG(nullptr) {
        m_fCurrentVelocity[REAR_LEFT_WHEEL]     = 0.0;
        m_fCurrentVelocity[REAR_RIGHT_WHEEL]    = 0.0;
        m_fCurrentVelocity[FRONT_LEFT_WHEEL]    = 0.0;
        m_fCurrentVelocity[FRONT_RIGHT_WHEEL]   = 0.0;
        m_fNoiseBias[REAR_LEFT_WHEEL]           = 1.0;
        m_fNoiseBias[REAR_RIGHT_WHEEL]          = 1.0;
        m_fNoiseBias[FRONT_LEFT_WHEEL]          = 1.0;
        m_fNoiseBias[FRONT_RIGHT_WHEEL]         = 1.0;
        m_fNoiseFactorAvg[REAR_LEFT_WHEEL]      = 1.0;
        m_fNoiseFactorAvg[REAR_RIGHT_WHEEL]     = 1.0;
        m_fNoiseFactorAvg[FRONT_LEFT_WHEEL]     = 1.0;
        m_fNoiseFactorAvg[FRONT_RIGHT_WHEEL]    = 1.0;
        m_fNoiseFactorStdDev[REAR_LEFT_WHEEL]   = 0.0;
        m_fNoiseFactorStdDev[REAR_RIGHT_WHEEL]  = 0.0;
        m_fNoiseFactorStdDev[FRONT_LEFT_WHEEL]  = 0.0;
        m_fNoiseFactorStdDev[FRONT_RIGHT_WHEEL] = 0.0;
    }

    /****************************************/
    /****************************************/

    void CAckermannSteeringDefaultActuator::SetRobot(CComposableEntity& c_entity) {
        try {
            m_pcAckermannWheeledEntity = &(c_entity.GetComponent<CAckermannWheeledEntity>("wheels"));
            if (m_pcAckermannWheeledEntity->GetNumWheels() != 4) {
                THROW_ARGOSEXCEPTION("The Ackermann steering actuator can be associated only to a robot with 4 wheels");
            }
            m_pcAckermannWheeledEntity->Enable();
        } catch (CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error setting Ackermann steering actuator to entity \"" << c_entity.GetId() << "\"", ex);
        }
    }

    /****************************************/
    /****************************************/

#define CHECK_ATTRIBUTE(ATTR)                           \
    (NodeAttributeExists(t_tree, ATTR) ||               \
     NodeAttributeExists(t_tree, ATTR "_rear_left") ||  \
     NodeAttributeExists(t_tree, ATTR "_rear_right") || \
     NodeAttributeExists(t_tree, ATTR "_front_left") || \
     NodeAttributeExists(t_tree, ATTR "_front_right"))

#define PARSE_ATTRIBUTES(ATTR, VAR)                                                                            \
    GetNodeAttributeOrDefault<Real>(t_tree, ATTR, VAR[REAR_LEFT_WHEEL], VAR[REAR_LEFT_WHEEL]);                 \
    VAR[REAR_RIGHT_WHEEL]  = VAR[REAR_LEFT_WHEEL];                                                             \
    VAR[FRONT_LEFT_WHEEL]  = VAR[REAR_LEFT_WHEEL];                                                             \
    VAR[FRONT_RIGHT_WHEEL] = VAR[REAR_LEFT_WHEEL];                                                             \
    GetNodeAttributeOrDefault<Real>(t_tree, ATTR "_rear_left", VAR[REAR_LEFT_WHEEL], VAR[REAR_LEFT_WHEEL]);    \
    GetNodeAttributeOrDefault<Real>(t_tree, ATTR "_rear_right", VAR[REAR_RIGHT_WHEEL], VAR[REAR_RIGHT_WHEEL]); \
    GetNodeAttributeOrDefault<Real>(t_tree, ATTR "_front_left", VAR[FRONT_LEFT_WHEEL], VAR[FRONT_LEFT_WHEEL]); \
    GetNodeAttributeOrDefault<Real>(t_tree, ATTR "_front_right", VAR[FRONT_RIGHT_WHEEL], VAR[FRONT_RIGHT_WHEEL]);

#define PICK_BIAS(LRW) m_fNoiseBias[LRW##_WHEEL] = m_pcRNG->Gaussian(fNoiseBiasStdDev[LRW##_WHEEL], fNoiseBiasAvg[LRW##_WHEEL])

    void CAckermannSteeringDefaultActuator::Init(TConfigurationNode& t_tree) {
        try {
            CCI_DifferentialSteeringActuator::Init(t_tree);
            /* Check if any noise attribute was specified */
            bool bNoise =
                CHECK_ATTRIBUTE("bias_avg") ||
                CHECK_ATTRIBUTE("bias_stddev") ||
                CHECK_ATTRIBUTE("factor_avg") ||
                CHECK_ATTRIBUTE("factor_stddev");
            /* Handle noise attributes, if any */
            if (bNoise) {
                /* Create RNG */
                m_pcRNG = CRandom::CreateRNG("argos");
                /* Parse noise attributes */
                Real fNoiseBiasAvg[4];
                Real fNoiseBiasStdDev[4];
                PARSE_ATTRIBUTES("bias_avg", fNoiseBiasAvg);
                PARSE_ATTRIBUTES("bias_stddev", fNoiseBiasStdDev);
                PARSE_ATTRIBUTES("factor_avg", m_fNoiseFactorAvg);
                PARSE_ATTRIBUTES("factor_stddev", m_fNoiseFactorStdDev);
                /* Choose robot bias */
                PICK_BIAS(REAR_LEFT);
                PICK_BIAS(REAR_RIGHT);
                PICK_BIAS(FRONT_LEFT);
                PICK_BIAS(FRONT_RIGHT);
            }
        } catch (CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Initialization error in foot-bot steering actuator.", ex);
        }
    }

    /****************************************/
    /****************************************/

#define ADD_GAUSSIAN(LRW)                                                                           \
    (m_fNoiseFactorStdDev[LRW##_WHEEL] > 0.0 ? m_pcRNG->Gaussian(m_fNoiseFactorStdDev[LRW##_WHEEL], \
                                                                 m_fNoiseFactorAvg[LRW##_WHEEL])    \
                                             : m_fNoiseFactorAvg[LRW##_WHEEL])

#define ADD_NOISE(LRW)                     \
    m_fCurrentVelocity[LRW##_WHEEL] =      \
        ADD_GAUSSIAN(LRW) *                \
        (m_fCurrentVelocity[LRW##_WHEEL] + \
         m_fNoiseBias[LRW##_WHEEL]);

    void CAckermannSteeringDefaultActuator::SetSteeringAndThrottle(Real f_steering_ang,
                                                                   Real f_throttle_speed) {
        /* Convert speeds in m/s */
        m_fThrottleSpeed = f_throttle_speed * 0.01;
        m_fSteeringAngle = f_steering_ang;
        /* Apply noise only if the robot is in motion */
        if (m_pcRNG &&
            (f_throttle_speed != 0)) {
            ADD_NOISE(REAR_LEFT);
            ADD_NOISE(REAR_RIGHT);
            ADD_NOISE(FRONT_LEFT);
            ADD_NOISE(FRONT_RIGHT);
        }
    }

    /****************************************/
    /****************************************/

    void CAckermannSteeringDefaultActuator::Update() {
        m_pcAckermannWheeledEntity->SetSteeringAndThrottle(m_fSteeringAngle, m_fThrottleSpeed);
    }

    /****************************************/
    /****************************************/

    void CAckermannSteeringDefaultActuator::Reset() {
        /* Zero the speeds */
        m_fThrottleSpeed = 0.0;
        m_fSteeringAngle = 0.0;
        /*
         * Throw away two RNG Gaussian calls to make sure the RNG sequence is the same after resetting
         * These two calls were used to pick the bias in Init()
         */
        // TODO: why specifically 2 calls? Does that correspond to the number of wheels in the DiffSteering class?
        if (m_pcRNG) {
            m_pcRNG->Gaussian(1.0, 0.0);
            m_pcRNG->Gaussian(1.0, 0.0);
        }
    }

    /****************************************/
    /****************************************/

}

REGISTER_ACTUATOR(CAckermannSteeringDefaultActuator,
                  "ackermann_steering", "default",
                  "Carlo Pinciroli [ilpincy@gmail.com], Khai Yi Chin [khaiyichin@gmail.com]",
                  "1.0",
                  "The Ackermann steering actuator.",

                  "This actuator controls the steering and throttle of an Ackermann steering robot. For a\n"
                  "complete description of its usage, refer to the\n"
                  "ci_ackermann_steering_actuator.h file.\n\n"

                  "REQUIRED XML CONFIGURATION\n\n"
                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <ackermann_steering implementation=\"default\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"

                  "OPTIONAL XML CONFIGURATION\n\n"

                  "It is possible to specify noisy speed in order to match the characteristics\n"
                  "of the real robot. For each wheel, the noise model is as follows:\n\n"
                  "w = ideal wheel actuation (as set in the controller)\n"
                  "b = random bias from a Gaussian distribution\n"
                  "f = random factor from a Gaussian distribution\n"
                  "a = actual actuated value\n\n"
                  "a = f * (w + b)\n\n"
                  "You can configure the average and stddev of both the bias and the factor. This\n"
                  "can be done with the optional attributes: 'bias_avg', 'bias_stddev',\n"
                  "'factor_avg', and 'factor_stddev'. Bias attributes are expressed in m/s, while\n"
                  "factor attributes are dimensionless. If none of these attributed is specified,\n"
                  "no noise is added. If at least one of these attributed is specified, noise is\n"
                  "added and, for the non-specified attributes, the default value of 1 is used for\n"
                  "the '*_avg' attributes, while 0 is used for '*_stddev' attributes. Examples:\n\n"

                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <!-- Only the stddev of the bias\n"
                  "             Noise is on, other attributes are default -->\n"
                  "        <ackermann_steering implementation=\"default\"\n"
                  "                               bias_stddev=\"2\" />\n"
                  "        <!-- Only the stddev of the factor\n"
                  "             Noise is on, other attributes are default -->\n"
                  "        <ackermann_steering implementation=\"default\"\n"
                  "                               factor_stddev=\"4\" />\n"
                  "        <!-- All attributes set\n"
                  "             Noise is on, specified values are set -->\n"
                  "        <ackermann_steering implementation=\"default\"\n"
                  "                               bias_avg=\"1\"\n"
                  "                               bias_stddev=\"2\"\n"
                  "                               factor_avg=\"3\"\n"
                  "                               factor_stddev=\"4\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"

                  "The above examples set the same noise for all 4 wheels. If you want to set\n"
                  "different noise parameters for each wheel, append '_rear_left', '_rear_right',\n"
                  "'_front_left' and '_front_right' to the attribute names:\n\n"

                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <!-- Mix of wheel-specific attributes set\n"
                  "             Noise is on, specified values are set -->\n"
                  "        <ackermann_steering implementation=\"default\"\n"
                  "                               bias_avg_rear_left=\"1\"\n"
                  "                               bias_stddev_rear_right=\"2\"\n"
                  "                               factor_avg_front_left=\"3\"\n"
                  "                               factor_stddev_front_right=\"4\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"

                  "Wheel-specific attributes overwrite the values of non-wheel specific attributes.\n"
                  "So, if you set 'bias_avg' = 2 and then 'bias_avg_rear_left' = 3, the rear left wheel will\n"
                  "use 3 and the other three wheels will use 2.\n\n"
                  "Physics-engine-specific attributes that affect this actuator might also be\n"
                  "available. Check the documentation of the physics engine you're using for more\n"
                  "information.",
                  "Usable");