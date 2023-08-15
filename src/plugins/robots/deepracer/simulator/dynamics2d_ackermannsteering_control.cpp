#include "dynamics2d_ackermannsteering_control.h"

namespace argos {

    CDynamics2DAckermannSteeringControl::CDynamics2DAckermannSteeringControl(CDynamics2DEngine& c_engine,
                                                                             Real f_max_force,
                                                                             Real f_max_torque,
                                                                             Real f_interwheel_distance,
                                                                             Real f_wheelbase_distance,
                                                                             TConfigurationNode* t_node)
        : CDynamics2DVelocityControl(c_engine, f_max_force, f_max_torque, t_node),
          m_fInterwheelDistance(f_interwheel_distance), m_fWheelbaseDistance(f_wheelbase_distance) {}

    void CDynamics2DAckermannSteeringControl::SetSteeringAndThrottle(Real f_steering_ang,
                                                                     Real f_throttle_speed) {
        // Compute the linear and angular velocity of the body
        // https://www.xarg.org/book/kinematics/ackerman-steering/

        /*
         * THE ACKERMANN STEERING SYSTEM
         *
         * check https://www.xarg.org/book/kinematics/ackerman-steering/
         * for details
         *
         * Equations:
         *
         * w = (s / l) tan(p)
         * v = [ s cos(a),
         *       s sin(a)]
         *
         * where:
         *      a = body orientation
         *      w = body angular velocity
         *      v = body center linear velocity
         *      s = vehicle speed (wheel speed produced by throttle)
         *      p = steering angle
         *      l = wheel base
         */

        SetAngularVelocity((f_throttle_speed / m_fWheelbaseDistance) * ::tan(f_steering_ang));
        CVector2 cLinVel(f_throttle_speed * ::cos(m_ptControlledBody->a),
                         f_throttle_speed * ::sin(m_ptControlledBody->a));

        SetLinearVelocity(cLinVel);
    }

}