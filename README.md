# ARGoS3 - Deepracer AWS integration #

This repository contains the code that integrates the [ARGoS3 multi-robot simulator](http://www.argos-sim.info) and the [Deepracer AWS robot](https://aws.amazon.com/deepracer/).

This code allows you to simulate the Deepracer AWS in ARGoS3, and to run ARGoS3 controllers on the real Deepracer AWS.

# Compilation Instructions #

## To Simulate The Deepracer AWS in ARGoS3 ##

    $ mkdir build_sim
    $ cd build_sim
    $ cmake -DCMAKE_BUILD_TYPE=Release ../src
    $ make -j8
    
## To Run ARGoS3 Code on the Deepracer AWS ##

First, you need to install the [light toolchain provided by K-Team](http://ftp.k-team.com/KheperaIV/software/Gumstix%20COM%20Y/light_tools/poky-glibc-i686-khepera4-image-cortexa8hf-vfp-neon-toolchain-1.8.sh).

Next, you need to designate a folder where you'll install all the ARGoS-related binaries. Let's call this folder `${INSTALLPREFIX}`. Make sure the folder exists and it is writable by your user:

    $ mkdir -p ${INSTALLPREFIX}

To make ARGoS controllers work on the Deepracer AWS robot, you need to crosscompile the ARGoS core and the Deepracer AWS plugin. In the following, we will assume that you have three directories:

| Variable        | Meaning                                     |
|-----------------|---------------------------------------------|
| `INSTALLPREFIX` | Where the compile code is installed         |
| `ARGOS3PATH`    | Where the ARGoS3 core code is stored        |
| `KHIVPATH`      | Where the the code of this plugin is stored |

    $ mkdir build_dprcr
    $ cd build_dprcr
    $ cmake -DCMAKE_BUILD_TYPE=dprcr ../src
    $ make -j8
