cmake_minimum_required(VERSION 2.8.3)
project(formation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosmsg
  rospy
  std_msgs
  message_generation
)


generate_messages(
   DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
)

