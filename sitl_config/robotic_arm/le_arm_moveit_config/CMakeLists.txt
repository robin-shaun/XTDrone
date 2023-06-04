cmake_minimum_required(VERSION 2.8.3)
project(le_arm_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  trajectory_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(moveit_fk_demo src/moveit_fk_demo.cpp)
target_link_libraries(moveit_fk_demo ${catkin_LIBRARIES})

