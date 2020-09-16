cmake_minimum_required(VERSION 2.8.3)
project(ros_actor_cmd_pose_plugin_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
)

add_service_files(
  FILES
   ToggleActorWaving.srv
)

generate_messages(
   DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)
