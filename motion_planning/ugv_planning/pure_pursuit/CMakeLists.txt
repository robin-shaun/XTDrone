cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test_node src/test.cpp src/subscriber/path_subscriber.cpp)
target_link_libraries(${PROJECT_NAME}_test_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)