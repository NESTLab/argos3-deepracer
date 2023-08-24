if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(deepracer_diffusion MODULE deepracer_diffusion.h deepracer_diffusion.cpp)
endif(ARGOS_BUILD_FOR_SIMULATOR)
if(ARGOS_BUILD_FOR STREQUAL "dprcr")
  add_executable(deepracer_diffusion deepracer_diffusion.h deepracer_diffusion.cpp ${CMAKE_SOURCE_DIR}/plugins/robots/turtlebot3/real_robot/main.cpp)
endif(ARGOS_BUILD_FOR STREQUAL "dprcr")

target_link_libraries(deepracer_diffusion
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_deepracer
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)