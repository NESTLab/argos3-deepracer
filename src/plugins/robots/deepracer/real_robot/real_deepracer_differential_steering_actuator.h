#ifndef REAL_DEEPRACER_DIFFERENTIAL_STEERING_ACTUATOR_H
#define REAL_DEEPRACER_DIFFERENTIAL_STEERING_ACTUATOR_H

#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CRealDeepracerDifferentialSteeringActuator :
        public CCI_DifferentialSteeringActuator

        {
public:

    CRealDeepracerDifferentialSteeringActuator(std::shared_ptr<PublisherT>  pub);
    virtual ~CRealDeepracerDifferentialSteeringActuator();
    virtual void Do(Real f_elapsed_time);
    virtual void SetLinearVelocity(Real f_left_velocity,
                                   Real f_right_velocity);
};

#endif

};


#endif //ARGOS3_DEEPRACER_REAL_DEEPRACER_DIFFERENTIAL_STEERING_ACTUATOR_H
