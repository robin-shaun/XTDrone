cmake_minimum_required(VERSION 2.8.3)
project(gazebo_test_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  std_msgs
  object_msgs
  message_generation
)

find_package(gazebo REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES RecognizeGazeboObject.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   object_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_test_tools
  CATKIN_DEPENDS object_msgs gazebo_ros geometry_msgs roscpp std_msgs message_runtime
  DEPENDS gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(gazebo_test_tools
   src/cube_spawner.cpp
)

add_library(fake_object_recognizer
    src/FakeObjectRecognizer.cpp
)

## Declare a cpp executable
add_executable(cube_spawner src/cube_spawner_node.cpp)
add_executable(fake_object_recognizer_node src/fake_object_recognizer_node.cpp)
add_executable(fake_object_recognizer_cmd test/fake_object_recognizer_cmd.cpp)
add_executable(set_gazebo_physics_client src/SetGazeboPhysicsClient.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(gazebo_test_tools ${catkin_EXPORTED_TARGETS})
add_dependencies(cube_spawner ${catkin_EXPORTED_TARGETS})
add_dependencies(set_gazebo_physics_client ${catkin_EXPORTED_TARGETS})

add_dependencies(fake_object_recognizer gazebo_test_tools_generate_messages_cpp)
add_dependencies(fake_object_recognizer ${catkin_EXPORTED_TARGETS})

add_dependencies(fake_object_recognizer_node gazebo_test_tools_generate_messages_cpp)
add_dependencies(fake_object_recognizer_node ${catkin_EXPORTED_TARGETS})

add_dependencies(fake_object_recognizer_cmd gazebo_test_tools_generate_messages_cpp)
add_dependencies(fake_object_recognizer_cmd ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(gazebo_test_tools
   ${GAZEBO_LIBRARIES}
   ${catkin_LIBRARIES}
)

target_link_libraries(cube_spawner
   gazebo_test_tools
   ${catkin_LIBRARIES}
)

target_link_libraries(fake_object_recognizer
   ${catkin_LIBRARIES}
)

target_link_libraries(set_gazebo_physics_client
   ${catkin_LIBRARIES}
)

target_link_libraries(fake_object_recognizer_node
   fake_object_recognizer
   ${catkin_LIBRARIES}
)

target_link_libraries(fake_object_recognizer_cmd
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS gazebo_test_tools cube_spawner fake_object_recognizer set_gazebo_physics_client fake_object_recognizer_node fake_object_recognizer_cmd
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)


install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gazebo_test_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
