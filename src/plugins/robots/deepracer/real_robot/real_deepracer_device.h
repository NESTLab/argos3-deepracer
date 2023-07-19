#ifndef REAL_DEEPRACER_DEVICE_H
#define REAL_DEEPRACER_DEVICE_H
#include <signal.h>
#include <unistd.h>
#include <memory> //Not sure but I just include this for now
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace argos;

class CRealDeepracerDevice {
    CRealDeepracerDevice();
    virtual ~CRealDeepracerDevice() {}
    virtual void Do(Real f_elapsed_time) = 0;

};


#endif //REAL_DEEPRACER_DEVICE_H
