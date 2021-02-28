cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)
project(roboticsgroup_gazebo_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  control_toolbox
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
# Gazebo cxx flags should have all the required C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    control_toolbox
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(roboticsgroup_gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)
target_link_libraries(roboticsgroup_gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(roboticsgroup_gazebo_disable_link_plugin src/disable_link_plugin.cpp)
target_link_libraries(roboticsgroup_gazebo_disable_link_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS roboticsgroup_gazebo_mimic_joint_plugin roboticsgroup_gazebo_disable_link_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
