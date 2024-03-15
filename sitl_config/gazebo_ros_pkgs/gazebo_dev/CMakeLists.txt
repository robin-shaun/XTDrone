cmake_minimum_required(VERSION 3.0.2)
project(gazebo_dev)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS gazebo_dev-extras.cmake
)
