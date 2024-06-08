# ARGoS3 - AWS DeepRacer integration #

This repository contains the code that integrates the [ARGoS3 multi-robot simulator](http://www.argos-sim.info) and the [AWS DeepRacer robot](https://aws.amazon.com/deepracer/).

This code allows you to simulate the AWS DeepRacer in ARGoS3, and to run ARGoS3 controllers on the real AWS DeepRacer.

# Compilation Instructions #
Ideally, you arrive at this repository after visting the [NESTLab fork of the `aws-deepracer` repository](https://github.com/NESTLab/aws-deepracer). If not, please check out that repository carefully before proceeding.

## To Simulate the AWS DeepRacer in ARGoS3 ##
Clone this repository on your local machine, then do the following to install the plugin (note: you will need ARGoS3 installed).

    $ mkdir build_sim
    $ cd build_sim
    $ cmake -DCMAKE_BUILD_TYPE=Release ../src
    $ make
    $ sudo make install
    
## To Run ARGoS3 Code on the AWS DeepRacer ##
>:warning: If you are led here from the [NESTLab fork of the `aws-deepracer` repository](https://github.com/NESTLab/aws-deepracer), then the following instructions are not necessary. There should be a script from that repository that automates the installation of the plugin on the AWS DeepRacer.

The AWS DeepRacer runs a x86_64 architecture compute module with Ubuntu 20.04 loaded, so you can directly build the code onboard. To make ARGoS controllers work on the AWS DeepRacer robot, you need to install both the ARGoS core and the AWS DeepRacer plugin (this repository).

Designate a folder where you'll install all the ARGoS-related binaries. Let's call this folder `${INSTALLPREFIX}`. Make sure the folder exists and it is writable by your user:

    $ mkdir -p ${INSTALLPREFIX}

 In the following, we will assume that you have three directories:

| Variable        | Meaning                                     |
|-----------------|---------------------------------------------|
| `INSTALLPREFIX` | Where the compile code is installed         |
| `ARGOS3PATH`    | Where the ARGoS3 core code is stored        |
| `DPRCRPATH`     | Where the code of this plugin is stored     |

### Installing ARGoS ###

    $ cd ${ARGOS3PATH}
    $ mkdir build_dprcr
    $ cd build_dprcr
    $ cmake -DCMAKE_BUILD_TYPE=Release \
        -DARGOS_BUILD_FOR=dprcr \
        -DARGOS_DOCUMENTATION=OFF \
        -DARGOS_BUILD_NATIVE=ON \
        ../src
    $ make install
    $ ldconfig

### Installing the AWS DeepRacer plugin ###

    $ cd ${DPRCRPATH}
    $ mkdir build_dprcr
    $ cd build_dprcr
    $ cmake -DCMAKE_BUILD_TYPE=Release -DARGOS_BUILD_FOR=dprcr ../src
    $ make install
    $ ldconfig

# Dev Notes #
The following notes contain information on certain design and parameterization decisions in this plugin.

- The Ackermann steering class is implemented as a 4WD model, i.e., all 4 wheels are driven by the throttle. Because the wheels don't actually turn, this doesn't affect the dynamics; otherwise, having very sticky wheels that aim to rotate at the same speed (i.e., no slippage or differentials) could break the dynamics. Under the hood, the Ackermann wheeled entity class is simply a container to store the steering angle and throttle speed. These values are then used to compute the center of mass' linear and angular velocities, which are the only quantities that the Chipmunk physics engine cares about.
- Max & min steering and throttle speeds *(source : [`cmdvel_to_servo_node` package](https://github.com/aws-deepracer/aws-deepracer/tree/main/deepracer_nodes/cmdvel_to_servo_pkg))*:
    - Max forward speed = 4.0 m/s
    - Max back speed = -4.0 m/s
    - Max and mni steering angle = (-$\pi$/6, -$\pi$/6) rad (assumption: right handed rule)
- On the AWS DeepRacer physical vehicle a LiDAR sensor is mounted on the rear and tilted down by 6 degrees. It rotates at the angular velocity of 10 rotations per second and has a range of 15cm to 2m. It can detect objects behind and beside the host vehicle as well as tall objects unobstructed by the vehicle parts in the front. The angle and range are chosen to make the LiDAR unit less susceptible to environmental noise. *(source: [AWS](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-choose-race-type.html))*
