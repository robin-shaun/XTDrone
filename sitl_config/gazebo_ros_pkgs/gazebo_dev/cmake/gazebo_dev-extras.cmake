# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

message(STATUS "Gazebo version: ${GAZEBO_VERSION}")

# The following lines will tell catkin to add the Gazebo directories and libraries to the
# respective catkin_* cmake variables.
set(gazebo_dev_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS})
set(gazebo_dev_LIBRARY_DIRS ${GAZEBO_LIBRARY_DIRS})
set(gazebo_dev_LIBRARIES ${GAZEBO_LIBRARIES})

# Append gazebo CXX_FLAGS to CMAKE_CXX_FLAGS (c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
