if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(deepracer_drive_demo MODULE deepracer_drive_demo.h deepracer_drive_demo.cpp)
  add_library(deepracer_hello_world MODULE deepracer_hello_world.h deepracer_hello_world.cpp)
endif(ARGOS_BUILD_FOR_SIMULATOR)
if(ARGOS_BUILD_FOR STREQUAL "dprcr")
  add_executable(deepracer_drive_demo deepracer_drive_demo.h deepracer_drive_demo.cpp ${CMAKE_SOURCE_DIR}/plugins/robots/deepracer/real_robot/main.cpp)
  add_executable(deepracer_hello_world deepracer_hello_world.h deepracer_hello_world.cpp ${CMAKE_SOURCE_DIR}/plugins/robots/deepracer/real_robot/main.cpp)
endif(ARGOS_BUILD_FOR STREQUAL "dprcr")

target_link_libraries(deepracer_drive_demo
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_deepracer
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

target_link_libraries(deepracer_hello_world
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_deepracer
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)