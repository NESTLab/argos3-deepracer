if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(deepracer_diffusion MODULE deepracer_diffusion.h deepracer_diffusion.cpp)
  add_library(deepracer_camera MODULE deepracer_camera.h deepracer_camera.cpp)

  add_library(deepracer_hello_world MODULE deepracer_hello_world.h deepracer_hello_world.cpp)
endif(ARGOS_BUILD_FOR_SIMULATOR)
if(ARGOS_BUILD_FOR STREQUAL "dprcr")
  add_executable(deepracer_diffusion deepracer_diffusion.h deepracer_diffusion.cpp ${CMAKE_SOURCE_DIR}/plugins/robots/deepracer/real_robot/main.cpp)
  add_executable(deepracer_hello_world deepracer_hello_world.h deepracer_hello_world.cpp ${CMAKE_SOURCE_DIR}/plugins/robots/deepracer/real_robot/main.cpp)
endif(ARGOS_BUILD_FOR STREQUAL "dprcr")

target_link_libraries(deepracer_diffusion
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_deepracer
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

target_link_libraries(deepracer_hello_world
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_deepracer
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)