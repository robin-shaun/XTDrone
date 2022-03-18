cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosmsg
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
   UAVDataMsg.msg
   NeighborMsg.msg
   AllUAVData.msg
   AllNeighborMsg.msg
   CommVerify.msg
   UavComm.msg
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

