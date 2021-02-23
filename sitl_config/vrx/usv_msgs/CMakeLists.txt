cmake_minimum_required(VERSION 2.8.3)
project(usv_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

add_message_files(
   FILES
   RangeBearing.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_generation message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
