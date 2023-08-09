# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "plugins/robots/deepracer/CMakeFiles/argos3plugin_simulator_deepracer_autogen.dir/AutogenUsed.txt"
  "plugins/robots/deepracer/CMakeFiles/argos3plugin_simulator_deepracer_autogen.dir/ParseCache.txt"
  "plugins/robots/deepracer/argos3plugin_simulator_deepracer_autogen"
  )
endif()
