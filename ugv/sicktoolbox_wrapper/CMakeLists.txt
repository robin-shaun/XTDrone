cmake_minimum_required(VERSION 2.8.3)
project(sicktoolbox_wrapper)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs sicktoolbox rosconsole diagnostic_updater)

## System dependencies are found with CMake's conventions
#find_package(Threads)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp sensor_msgs sicktoolbox rosconsole diagnostic_updater
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

add_executable(sickld ros/sickld/sickld.cpp)
target_link_libraries(sickld ${catkin_LIBRARIES})

add_executable(sicklms ros/sicklms/sicklms.cpp)
target_link_libraries(sicklms ${catkin_LIBRARIES})

add_executable(print_scans standalone/print_scans.cpp)
target_link_libraries(print_scans ${catkin_LIBRARIES})

add_executable(time_scans standalone/time_scans.cpp)
target_link_libraries(time_scans ${catkin_LIBRARIES})

add_executable(log_scans standalone/log_scans.cpp)
target_link_libraries(log_scans ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS sickld sicklms print_scans time_scans
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hh"
#  PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   sick_test.vcg
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)