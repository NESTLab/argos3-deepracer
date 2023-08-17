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
    - make generic the imu sensor?
- ~~complete deepracer_measures file~~
- obtain mass, max force and max torque values for `dynamics2d_deepracer_model`.
    - what are the units of force and torque? also, [what is force for](https://github.com/ilpincy/argos3/blob/096d497565d47f0907e1ac7162143150bfff7181/src/plugins/simulator/physics_engines/dynamics2d/dynamics2d_velocity_control.cpp#L63)?
- complete the following classes (do global search on "TODO"s):
    - dynamics2d_deepracer_model
    - deepracer_entity: the camera and lidar entities to be added, shape of the robot
    - ~~lidar default sensor (may have been implemented by Nhi already)~~
- ~~remove the imu equipped entity (based on the [positioning sensor](https://github.com/ilpincy/argos3/blob/master/src/plugins/robots/generic/simulator/positioning_default_sensor.h) we can just use the sensor without the entity)~~


## Unorganized notes (TODO: clean up before release)
- The Ackermann steering class is implemented as a 4WD model, i.e., all 4 wheels are driven by the throttle. Because the wheels don't actually turn, this doesn't affect the dynamics; otherwise, having very sticky wheels that aim to rotate at the same speed (i.e., no slippage or differentials) could break the dynamics. Under the hood, the Ackermann wheeled entity class is simply a container to store the steering angle and throttle speed. These values are then used to compute the center of mass' linear and angular velocities, which are the only quantities that the Chipmunk physics engine cares about.
- Max & min steering and throttle speeds:
    - What is the max forward speed? 4.0 m/s from the `cmdvel_to_servo_node` repo
    - Max back speed = -4.0 m/s
    - max steer = pi/6 rad, min steer = -pi/6 rad (assumption: right handed rule)
- (from https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-choose-race-type.html) On the AWS DeepRacer physical vehicle a LiDAR sensor is mounted on the rear and tilted down by 6 degrees. It rotates at the angular velocity of 10 rotations per second and has a range of 15cm to 2m. It can detect objects behind and beside the host vehicle as well as tall objects unobstructed by the vehicle parts in the front. The angle and range are chosen to make the LiDAR unit less susceptible to environmental noise.