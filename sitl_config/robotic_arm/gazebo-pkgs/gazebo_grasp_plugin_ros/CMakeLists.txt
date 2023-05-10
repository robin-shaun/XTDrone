cmake_minimum_required(VERSION 3.0.2)
project(gazebo_grasp_plugin_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_grasp_plugin
  message_generation
  roscpp
)

find_package(gazebo REQUIRED)

################################################
## Messages
################################################

add_message_files(
  FILES GazeboGraspEvent.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_grasp_plugin_ros
 CATKIN_DEPENDS gazebo_grasp_plugin message_runtime roscpp
 DEPENDS gazebo
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_republisher_node src/grasp_event_republisher.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_republisher_node"
set_target_properties(${PROJECT_NAME}_republisher_node PROPERTIES OUTPUT_NAME grasp_event_republisher PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_republisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_republisher_node gazebo_grasp_plugin_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_republisher_node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_republisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
