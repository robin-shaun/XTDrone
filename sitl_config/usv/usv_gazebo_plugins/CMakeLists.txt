cmake_minimum_required(VERSION 2.8.3)
project(usv_gazebo_plugins)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

find_package(catkin REQUIRED COMPONENTS gazebo_dev roscpp message_generation xacro wave_gazebo_plugins usv_msgs)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime gazebo_dev roscpp wave_gazebo_plugins
)

# Plugins require c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(buoyancy_gazebo_plugin
  src/buoyancy_gazebo_plugin.cc
  src/shape_volume.cc
  src/polyhedron_volume.cc
)
target_link_libraries(buoyancy_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS buoyancy_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_dynamics_plugin
  src/usv_gazebo_dynamics_plugin.cc
)
target_link_libraries(usv_gazebo_dynamics_plugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS usv_gazebo_dynamics_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_thrust_plugin
  src/usv_gazebo_thrust_plugin.cc
)
target_link_libraries(usv_gazebo_thrust_plugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS usv_gazebo_thrust_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_wind_plugin
  src/usv_gazebo_wind_plugin.cc
)
target_link_libraries(usv_gazebo_wind_plugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS usv_gazebo_wind_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Declare a C++ library
add_library(usv_gazebo_acoustic_pinger_plugin
  src/acoustic_pinger_plugin.cc
)
add_dependencies(usv_gazebo_acoustic_pinger_plugin usv_msgs_generate_messages_cpp)
target_link_libraries(usv_gazebo_acoustic_pinger_plugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS usv_gazebo_acoustic_pinger_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# Generate demo world files from xacro and install
xacro_add_files(
  worlds/buoyancy_plugin_demo.world.xacro
  ${XACRO_INORDER} INSTALL DESTINATION worlds
)
install(DIRECTORY worlds/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

if(CATKIN_ENABLE_TESTING)
  # buoyancy plugin test
  catkin_add_gtest(buoyancy_plugin_test test/buoyancy_test.cc)
  target_link_libraries(buoyancy_plugin_test buoyancy_gazebo_plugin)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
