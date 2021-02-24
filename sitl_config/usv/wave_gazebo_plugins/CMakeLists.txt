cmake_minimum_required(VERSION 2.8.3)
project(wave_gazebo_plugins)

###############################################################################
# Compile as C++11, supported in ROS Kinetic and newer

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For this package set as C++14
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

###############################################################################
# Other dependencies...

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

############################################################################### 
# Catkin...

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    Hydrodynamics
    WavefieldModelPlugin
    WavefieldVisualPlugin
    wavegauge_plugin
  DEPENDS
    gazebo_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-MATHS_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-MATHS_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
  )

############################################################################### 
# Libraries...

# Hydrodynamics
add_library(Hydrodynamics
  SHARED
    src/Gazebo.cc
    src/Geometry.cc
    src/PhysicalConstants.cc
    src/Physics.cc
    src/Utilities.cc
    src/Wavefield.cc
    src/WavefieldEntity.cc
)

target_link_libraries(Hydrodynamics
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_compile_options(Hydrodynamics PRIVATE "-Wno-unknown-pragmas")

list(APPEND WAVE_GAZEBO_LIBRARIES_LIST Hydrodynamics)

############################################################################### 
# Plugins...

# WavefieldModelPlugin
add_library(WavefieldModelPlugin
  SHARED
    src/WavefieldModelPlugin.cc
)

target_link_libraries(WavefieldModelPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(WavefieldModelPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND WAVE_GAZEBO_PLUGINS_LIST WavefieldModelPlugin)

# WavefieldVisualPlugin
add_library(WavefieldVisualPlugin
  SHARED
    src/WavefieldVisualPlugin.cc
)

target_link_libraries(WavefieldVisualPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)

target_compile_options(WavefieldVisualPlugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND WAVE_GAZEBO_PLUGINS_LIST WavefieldVisualPlugin)


# WavegaugePlugin
add_library(wavegauge_plugin
  SHARED
    src/wavegauge_plugin.cc
)

target_link_libraries(wavegauge_plugin
  WavefieldModelPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)

add_dependencies(wavegauge_plugin WavefieldModelPlugin)

target_compile_options(wavegauge_plugin PRIVATE "-Wno-unknown-pragmas")

list(APPEND WAVE_GAZEBO_PLUGINS_LIST wavegauge_plugin)

############################################################################### 
# Executables...


############################################################################### 
# Tests...
# 
# Useful notes on running catkin tests:
# http://www.personalrobotics.ri.cmu.edu/software/unit-testing
# 

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(UNIT_Wavefield_TEST src/Wavefield_TEST.cc)
  target_link_libraries(UNIT_Wavefield_TEST ${WAVE_GAZEBO_LIBRARIES_LIST})
  target_compile_options(UNIT_Wavefield_TEST PRIVATE "-Wno-unknown-pragmas")

endif()

############################################################################### 
# Install

install(TARGETS ${WAVE_GAZEBO_LIBRARIES_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${WAVE_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${WAVE_GAZEBO_EXECUTABLES_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)
