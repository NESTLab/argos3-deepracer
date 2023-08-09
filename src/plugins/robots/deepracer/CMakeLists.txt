#
# deepracer headers
#
# argos3/plugins/robots/deepracer/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_CONTROLINTERFACE
  # control_interface/ci_deepracer_camera_sensor.h
   control_interface/ci_deepracer_imu_sensor.h
  control_interface/ci_deepracer_lidar_sensor.h)
if(BUZZ_FOUND)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_CONTROLINTERFACE
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_CONTROLINTERFACE}
    control_interface/buzz_controller_deepracer.h) # doesn't work for now
endif(BUZZ_FOUND)
# argos3/plugins/robots/deepracer/simulator
# if(ARGOS_BUILD_FOR_SIMULATOR)
#     set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_SIMULATOR
#     simulator/dynamics2d_deepracer_model.h
#     simulator/deepracer_entity.h
#     simulator/deepracer_light_rotzonly_sensor.h
#     simulator/deepracer_measures.h
#     simulator/deepracer_ground_rotzonly_sensor.h
#     simulator/deepracer_lidar_default_sensor.h
#     simulator/deepracer_proximity_default_sensor.h
#     simulator/deepracer_ultrasound_default_sensor.h)
# endif(ARGOS_BUILD_FOR_SIMULATOR)
# argos3/plugins/robots/deepracer/real_robot
if(ARGOS_BUILD_FOR STREQUAL "dprcr")
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_REALROBOT
    real_robot/real_deepracer.h
    real_robot/real_deepracer_device.h
  #     real_robot/real_deepracer_camera_sensor.h
       real_robot/real_deepracer_imu_sensor.h
    real_robot/real_deepracer_lidar_sensor.h)
endif(ARGOS_BUILD_FOR STREQUAL "dprcr")

#
# deepracer sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_CONTROLINTERFACE}
#   control_interface/ci_deepracer_camera_sensor.cpp
   control_interface/ci_deepracer_imu_sensor.cpp
  control_interface/ci_deepracer_lidar_sensor.cpp)
if(BUZZ_FOUND)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER}
    control_interface/buzz_controller_deepracer.cpp) # no buzz controller written yet
endif(BUZZ_FOUND)

# if(ARGOS_BUILD_FOR_SIMULATOR)
#   set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER
#     ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER}
#     ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_SIMULATOR}
#     simulator/dynamics2d_deepracer_model.cpp
#     simulator/deepracer_entity.cpp
#     simulator/deepracer_light_rotzonly_sensor.cpp
#     simulator/deepracer_measures.cpp
#     simulator/deepracer_ground_rotzonly_sensor.cpp
#     simulator/deepracer_lidar_default_sensor.cpp
#     simulator/deepracer_proximity_default_sensor.cpp
#     simulator/deepracer_ultrasound_default_sensor.cpp)
#   # Compile the graphical visualization only if the necessary libraries have been found
#   if(ARGOS_QTOPENGL_FOUND)
#     include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
#     set(ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_SIMULATOR
#       ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_SIMULATOR}
#       simulator/qtopengl_deepracer.h)
#     set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER
#       ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER}
#       simulator/qtopengl_deepracer.h
#       simulator/qtopengl_deepracer.cpp)
#   endif(ARGOS_QTOPENGL_FOUND)
# endif(ARGOS_BUILD_FOR_SIMULATOR)
if(ARGOS_BUILD_FOR STREQUAL "dprcr")
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_REALROBOT}
    real_robot/real_deepracer.cpp
    real_robot/real_deepracer_device.cpp
#     real_robot/real_deepracer_camera_sensor.cpp
     real_robot/real_deepracer_imu_sensor.cpp
    real_robot/real_deepracer_lidar_sensor.cpp)
endif(ARGOS_BUILD_FOR STREQUAL "dprcr")

#
# Create deepracer plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_deepracer SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_DEEPRACER})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_deepracer
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_deepracer
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)
  if(ARGOS_QTOPENGL_FOUND)
    target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_deepracer argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
  endif(ARGOS_QTOPENGL_FOUND)
else(ARGOS_BUILD_FOR_SIMULATOR)
  ament_target_dependencies(argos3plugin_${ARGOS_BUILD_FOR}_deepracer rclcpp std_msgs sensor_msgs) # used to handle ROS 2 dependencies
endif(ARGOS_BUILD_FOR_SIMULATOR)
if(BUZZ_FOUND)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_deepracer argos3plugin_${ARGOS_BUILD_FOR}_buzz ${BUZZ_LIBRARY})
endif(BUZZ_FOUND)


#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/deepracer/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_SIMULATOR} DESTINATION include/argos3/plugins/robots/deepracer/simulator)
  install(FILES simulator/deepracer_texture_bottom.png simulator/deepracer_texture_side.png simulator/deepracer_texture_top.png DESTINATION include/argos3/plugins/simulator/visualizations/qt-opengl/textures)
else(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_DEEPRACER_REALROBOT} DESTINATION include/argos3/plugins/robots/deepracer/real_robot)
  install(FILES real_robot/main.cpp DESTINATION include/argos3/plugins/robots/deepracer/real_robot)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_deepracer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)