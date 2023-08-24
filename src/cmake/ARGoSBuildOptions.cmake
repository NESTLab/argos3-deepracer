#
# What is ARGoS being built for?
# Accepted values: "simulator" or a robot name (lowercase)
#
if(NOT DEFINED ARGOS_BUILD_FOR)
  # Variable was not set, set to default value
  set(ARGOS_BUILD_FOR "simulator" CACHE STRING "What is ARGoS being built for? \"simulator\" or a robot name (lowercase)")
else(NOT DEFINED ARGOS_BUILD_FOR)
  # Variable was set, make it public
  set(ARGOS_BUILD_FOR ${ARGOS_BUILD_FOR} CACHE STRING "What is ARGoS being built for? \"simulator\" or a robot name (lowercase)")
endif(NOT DEFINED ARGOS_BUILD_FOR)
# Set a macro according to value set in ARGOS_BUILD_FOR
add_definitions(-DARGOS_${ARGOS_BUILD_FOR}_BUILD)
# Create a convenience variable for checks in the CMake files
if(ARGOS_BUILD_FOR STREQUAL "simulator")
  set(ARGOS_BUILD_FOR_SIMULATOR TRUE)
else(ARGOS_BUILD_FOR STREQUAL "simulator")
  set(ARGOS_BUILD_FOR_SIMULATOR FALSE)
endif(ARGOS_BUILD_FOR STREQUAL "simulator")

#
# Optimize code for current platform?
#
if(NOT DEFINED ARGOS_BUILD_NATIVE)
  option(ARGOS_BUILD_NATIVE "ON -> compile with platform-specific optimizations, OFF -> compile to portable binary" OFF)
endif(NOT DEFINED ARGOS_BUILD_NATIVE)

#
# If building for real robot, check if ROS 2 is installed
#
if(NOT ARGOS_BUILD_FOR_SIMULATOR)
  # Check to see if ros 2 has been installed
  find_package(rclcpp) # try to find rclcpp (representative ROS 2 library)
  find_package(ament_cmake)
  find_package(std_msgs)
  find_package(std_srvs)
  find_package(sensor_msgs)
  find_package(deepracer_interfaces_pkg) # find the aws deepracer interface package
  include_directories(/opt/ros/foxy/include) # TODO: need to find a better way to do this (or perhaps this is the best?)
  if(NOT rclcpp_FOUND)
    message(FATAL_ERROR "Cannot find rclcpp library, please ensure that ROS 2 is installed")
  endif(NOT rclcpp_FOUND)
endif(NOT ARGOS_BUILD_FOR_SIMULATOR)