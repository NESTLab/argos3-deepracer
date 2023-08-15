# argos3-deepracer
SWARM ? Argos 
<br />Argos ? Deepracer- (tbc)

## TO-DOs
- submit PR to migrate classes to the `argos3` repo.
    - `plugins/robots/deepracer/control_interface/ci_ackermann_steering_actuator.*` -> `plugins/robots/generic/control_interface/ci_ackermann_steering_actuator.*`
    - `plugins/robots/deepracer/simulator/ackermann_steering_default_actuator.*` -> `plugins/robots/generic/simulator/ackermann_steering_default_actuator.*`
    - `plugins/robots/deepracer/simulator/dynamics2d_ackermannsteering_control.*` -> `plugins/simulator/physics_engines/dynamics2d/dynamics2d_ackermannsteering_control.*`
    - `plugins/robots/deepracer/simulator/deepracer_imu_sensor_equipped_entity.*` -> `plugins/simulator/entities/imu_sensor_equipped_entity.*`
    - `plugins/robots/deepracer/simulator/ackermann_wheeled_entity.*` -> `plugins/simulator/entities/ackermann_wheeled_entity.*`
- complete deepracer_measures file
- obtain mass, max force and max torque values for `dynamics2d_deepracer_model`.
- complete the following classes (do global search on "TODO"s):
    - dynamics2d_deepracer_model


## Unorganized notes (TODO: clean up before release)
- The Ackermann steering class is implemented as a 4WD model, i.e., all 4 wheels are driven by the throttle. Because the wheels don't actually turn, this doesn't affect the dynamics; otherwise, having very sticky wheels that aim to rotate at the same speed (i.e., no slippage or differentials) could break the dynamics. Under the hood, the Ackermann wheeled entity class is simply a container to store the steering angle and throttle speed. These values are then used to compute the center of mass' linear and angular velocities, which are the only quantities that the Chipmunk physics engine cares about.
- Max & min steering and throttle speeds:
    - What is the max forward speed? 4.0 m/s from the `cmdvel_to_servo_node` repo
    - Max back speed = -4.0 m/s
    - max steer = pi/6 rad, min steer = -pi/6 rad (assumption: right handed rule)