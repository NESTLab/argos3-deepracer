#ifndef DYNAMICS2D_ACKERMANNSTEERING_CONTROL_H
#define DYNAMICS2D_ACKERMANNSTEERING_CONTROL_H

namespace argos {
    class CDynamics2DEngine;
}

#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_velocity_control.h>

namespace argos {

    class CDynamics2DAckermannSteeringControl : public CDynamics2DVelocityControl {
    public:

        CDynamics2DAckermannSteeringControl(CDynamics2DEngine& c_engine,
                                            Real f_max_force,
                                            Real f_max_torque,
                                            Real f_interwheel_distance,
                                            Real f_wheelbase_distance,
                                            TConfigurationNode* t_node = NULL);

        virtual ~CDynamics2DAckermannSteeringControl() {}

        void SetSteeringAndThrottle(Real f_normalized_steering_ang,
                                    Real f_normalized_throttle);

        inline Real GetWheelbaseDistance() const {
            return m_fWheelbaseDistance;
        }

        inline Real GetInterwheelDistance() const {
            return m_fInterwheelDistance;
        }

    private:

        Real m_fWheelbaseDistance;

        Real m_fInterwheelDistance;
    };

}

#endif