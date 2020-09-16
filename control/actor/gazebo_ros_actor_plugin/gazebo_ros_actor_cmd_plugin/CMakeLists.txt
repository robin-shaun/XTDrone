cmake_minimum_required(VERSION 2.8.3)
project(ros_actor_cmd_pose_plugin)
set(CMAKE_CXX_STANDARD 11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  gazebo_plugins
  ros_actor_cmd_pose_plugin_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ActorPluginRos.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    gazebo_plugins
)

install (
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY res
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
