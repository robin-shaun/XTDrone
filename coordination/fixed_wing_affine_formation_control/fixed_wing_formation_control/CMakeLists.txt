cmake_minimum_required(VERSION 3.0.2)
project(fixed_wing_formation_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  std_srvs
  tf2_eigen
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    FWstates.msg
    FWcmd.msg
    Formation_control_states.msg
    Fwmonitor.msg
    Fw_cmd_mode.msg
    Fw_current_mode.msg
    Leaderstates.msg
    )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
  geometry_msgs   mavros_msgs   nav_msgs   sensor_msgs   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES fixed_wing_formation_control
  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs message_generation message_runtime nav_msgs roscpp rospy sensor_msgs serial std_msgs std_srvs tf2_eigen tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

################################################## 库生成声明 #################################################################

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fixed_wing_formation_control.cpp
# )

########### TECS控制器库生成声明 #############
add_library(tecs
    src/fixed_wing_lib/vertical_controller/tecs.cpp
    )

########### 编队控制器库生成声明 #############
add_library(formation_controller
    src/formation_controller/formation_controller.cpp
    )
add_library(abs_formation_controller
    src/formation_controller/abs_formation_controller.cpp
    )

########### 无人机状态打包器库生成声明 #############
add_library(pack_fw_states
    src/pack_fw_states/pack_fw_states.cpp
    )

########### 无人机模式切换库生成声明 #############
add_library(switch_fw_mode
    src/switch_fw_mode/switch_fw_mode.cpp
    )

################## 编队控制节点 #####################
add_library(task_main
    src/task_main/task_main.cpp
    )


################################################ 库文件链接 #############################################################

target_link_libraries(formation_controller
    tecs
    )
target_link_libraries( abs_formation_controller
    formation_controller
    )
target_link_libraries( task_main
    abs_formation_controller)


################################################ 仿真节点生成声明 #############################################################

###################################### 0号无人机-长机 ###########################################

add_executable(follower0_main src/multi_uav_sim/followers_main/follower0_main.cpp)
target_link_libraries(follower0_main
    ${catkin_LIBRARIES}
    )
target_link_libraries(follower0_main
    task_main
    )

########## 0号消息打包节点 #############

add_executable(pack_fw0_states src/multi_uav_sim/pack_fws_states/pack_fw0_states.cpp)
target_link_libraries(pack_fw0_states
    ${catkin_LIBRARIES}
    )
target_link_libraries(pack_fw0_states
    pack_fw_states
    )

########## 0号模式切换节点 #############

add_executable(switch_fw0_mode src/multi_uav_sim/switch_fws_mode/switch_fw0_mode.cpp)
target_link_libraries(switch_fw0_mode
    ${catkin_LIBRARIES}
    )
target_link_libraries(switch_fw0_mode
    switch_fw_mode
    )

###################################### 1号无人机-长机 ###########################################

add_executable(follower1_main src/multi_uav_sim/followers_main/follower1_main.cpp)
target_link_libraries(follower1_main
    ${catkin_LIBRARIES}
    )
target_link_libraries(follower1_main
    task_main
    )

########## 1号消息打包节点 #############

add_executable(pack_fw1_states src/multi_uav_sim/pack_fws_states/pack_fw1_states.cpp)
target_link_libraries(pack_fw1_states
    ${catkin_LIBRARIES}
    )
target_link_libraries(pack_fw1_states
    pack_fw_states
    )

########## 1号模式切换节点 #############

add_executable(switch_fw1_mode src/multi_uav_sim/switch_fws_mode/switch_fw1_mode.cpp)
target_link_libraries(switch_fw1_mode
    ${catkin_LIBRARIES}
    )
target_link_libraries(switch_fw1_mode
    switch_fw_mode
    )    
###################################### 2号无人机-长机 ###########################################

add_executable(follower2_main src/multi_uav_sim/followers_main/follower2_main.cpp)
target_link_libraries(follower2_main
    ${catkin_LIBRARIES}
    )
target_link_libraries(follower2_main
    task_main
    )

########## 2号消息打包节点 #############

add_executable(pack_fw2_states src/multi_uav_sim/pack_fws_states/pack_fw2_states.cpp)
target_link_libraries(pack_fw2_states
    ${catkin_LIBRARIES}
    )
target_link_libraries(pack_fw2_states
    pack_fw_states
    )

########## 2号模式切换节点 #############

add_executable(switch_fw2_mode src/multi_uav_sim/switch_fws_mode/switch_fw2_mode.cpp)
target_link_libraries(switch_fw2_mode
    ${catkin_LIBRARIES}
    )
target_link_libraries(switch_fw2_mode
    switch_fw_mode
    )

###################################### 3号无人机-僚机 ###########################################

add_executable(follower3_main src/multi_uav_sim/followers_main/follower3_main.cpp)
target_link_libraries(follower3_main
    ${catkin_LIBRARIES}
    )
target_link_libraries(follower3_main
    task_main
    )

########## 3号消息打包节点 #############

add_executable(pack_fw3_states src/multi_uav_sim/pack_fws_states/pack_fw3_states.cpp)
target_link_libraries(pack_fw3_states
    ${catkin_LIBRARIES}
    )
target_link_libraries(pack_fw3_states
    pack_fw_states
    )

########## 3号模式切换节点 #############

add_executable(switch_fw3_mode src/multi_uav_sim/switch_fws_mode/switch_fw3_mode.cpp)
target_link_libraries(switch_fw3_mode
    ${catkin_LIBRARIES}
    )
target_link_libraries(switch_fw3_mode
    switch_fw_mode
    )

###################################### 4号无人机-僚机 ###########################################

add_executable(follower4_main src/multi_uav_sim/followers_main/follower4_main.cpp)
target_link_libraries(follower4_main
    ${catkin_LIBRARIES}
    )
target_link_libraries(follower4_main
    task_main
    )

########## 4号消息打包节点 #############

add_executable(pack_fw4_states src/multi_uav_sim/pack_fws_states/pack_fw4_states.cpp)
target_link_libraries(pack_fw4_states
    ${catkin_LIBRARIES}
    )
target_link_libraries(pack_fw4_states
    pack_fw_states
    )

########## 4号模式切换节点 #############

add_executable(switch_fw4_mode src/multi_uav_sim/switch_fws_mode/switch_fw4_mode.cpp)
target_link_libraries(switch_fw4_mode
    ${catkin_LIBRARIES}
    )
target_link_libraries(switch_fw4_mode
    switch_fw_mode
    )

###################################### 5号无人机-僚机 ###########################################

add_executable(follower5_main src/multi_uav_sim/followers_main/follower5_main.cpp)
target_link_libraries(follower5_main
    ${catkin_LIBRARIES}
    )
target_link_libraries(follower5_main
    task_main
    )

########## 5号消息打包节点 #############

add_executable(pack_fw5_states src/multi_uav_sim/pack_fws_states/pack_fw5_states.cpp)
target_link_libraries(pack_fw5_states
    ${catkin_LIBRARIES}
    )
target_link_libraries(pack_fw5_states
    pack_fw_states
    )

########## 5号模式切换节点 #############

add_executable(switch_fw5_mode src/multi_uav_sim/switch_fws_mode/switch_fw5_mode.cpp)
target_link_libraries(switch_fw5_mode
    ${catkin_LIBRARIES}
    )
target_link_libraries(switch_fw5_mode
    switch_fw_mode
    )
    
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/fixed_wing_formation_control_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fixed_wing_formation_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
