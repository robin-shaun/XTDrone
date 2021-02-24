cmake_minimum_required(VERSION 2.8.3)
project(vrx_gazebo)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

# For Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()
# Use OLD policy to be able to use *_generate_services_cpp before they exist,
# without error.
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 OLD)
endif()

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  geographic_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  wamv_gazebo
  wave_gazebo
  xacro
)

########################
## Message generation ##
########################

add_message_files(
  FILES
  Task.msg
  Contact.msg
)

add_service_files(
  FILES
  ColorSequence.srv
)

# Python scripts setup
catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS wamv_gazebo wave_gazebo xacro gazebo_dev geographic_msgs std_msgs message_runtime
  LIBRARIES scoring_plugin
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}") 

include_directories(include ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_definitions(${QT_DEFINITIONS})

# Library for displaying waypoint markers
add_library(waypoint_markers src/waypoint_markers.cc)
target_link_libraries(waypoint_markers ${catkin_LIBRARIES})
add_dependencies(waypoint_markers ${catkin_EXPORTED_TARGETS})
install(TARGETS waypoint_markers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plugin for setting color of light buoy.
add_library(light_buoy_plugin src/light_buoy_plugin.cc)
target_link_libraries(light_buoy_plugin
  ${catkin_LIBRARIES} 
  light_buoy_colors_msgs
)
add_dependencies(light_buoy_plugin ${catkin_EXPORTED_TARGETS}
  light_buoy_colors_msgs
)
install(TARGETS light_buoy_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plugin for setting color and shape of a placard.
add_library(placard_plugin src/placard_plugin.cc)
target_link_libraries(placard_plugin
  ${catkin_LIBRARIES}
  dock_placard_msgs
)
add_dependencies(placard_plugin
  ${catkin_EXPORTED_TARGETS}
  dock_placard_msgs
)
install(TARGETS placard_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# A generic scoring plugin.
add_library(scoring_plugin src/scoring_plugin.cc)
target_link_libraries(scoring_plugin
  ${catkin_LIBRARIES}
)
add_dependencies(scoring_plugin
  ${catkin_EXPORTED_TARGETS}
  vrx_gazebo_generate_messages_cpp
  vrx_gazebo_generate_services_cpp
)
install(TARGETS scoring_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plugin for scoring the navigation challenge task.
add_library(navigation_scoring_plugin src/navigation_scoring_plugin.cc)
target_link_libraries(navigation_scoring_plugin
  ${catkin_LIBRARIES}
  scoring_plugin)
add_dependencies(navigation_scoring_plugin ${catkin_EXPORTED_TARGETS})
install(TARGETS navigation_scoring_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plugin for scoring the station keeping challenge task.
add_library(stationkeeping_scoring_plugin src/stationkeeping_scoring_plugin.cc)
target_link_libraries(stationkeeping_scoring_plugin
  ${catkin_LIBRARIES}
  scoring_plugin
  waypoint_markers)
add_dependencies(stationkeeping_scoring_plugin ${catkin_EXPORTED_TARGETS})
install(TARGETS stationkeeping_scoring_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plugin for scoring the waypoint navigation challenge task.
add_library(wayfinding_scoring_plugin src/wayfinding_scoring_plugin.cc)
target_link_libraries(wayfinding_scoring_plugin
  ${catkin_LIBRARIES}
  scoring_plugin
  waypoint_markers)
add_dependencies(wayfinding_scoring_plugin ${catkin_EXPORTED_TARGETS})
install(TARGETS wayfinding_scoring_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plugin for scoring the scan and dock task.
add_library(scan_dock_scoring_plugin src/scan_dock_scoring_plugin.cc)
target_link_libraries(scan_dock_scoring_plugin
  ${catkin_LIBRARIES}
  scoring_plugin 
  light_buoy_colors_msgs
  dock_placard_msgs
  )
add_dependencies(scan_dock_scoring_plugin
  ${catkin_EXPORTED_TARGETS}
  light_buoy_colors_msgs
  dock_placard_msgs
)
install(TARGETS scan_dock_scoring_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(${GAZEBO_MAJOR_VERSION} GREATER 7)
  # Plugin for Task Info GUI Overlay
  add_library(gui_task_widget SHARED ${headers_MOC}
         	src/gui_task_widget.cc
  )

  target_link_libraries(gui_task_widget 
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${QT_LIBRARIES}
  )
  qt5_use_modules(gui_task_widget Widgets)
  add_dependencies(gui_task_widget ${catkin_EXPORTED_TARGETS})
  install(TARGETS gui_task_widget
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# Plugin for scoring the perception task.
add_library(perception_scoring_plugin src/perception_scoring_plugin.cc)
target_link_libraries(perception_scoring_plugin
  ${catkin_LIBRARIES}
  scoring_plugin)
add_dependencies(perception_scoring_plugin ${catkin_EXPORTED_TARGETS})
install(TARGETS perception_scoring_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Dock base files that need to be processed with erb
set(dock_base_erb_files
  models/dock_2016_base/model.sdf.erb
  models/dock_2018_base/model.sdf.erb
  models/dock_2016_base_dynamic/model.sdf.erb
  models/dock_2018_base_dynamic/model.sdf.erb
)

# Dock files that need to be processed with erb
set(dock_erb_files
  models/dock_2016/model.sdf.erb
  models/dock_2018/model.sdf.erb
  models/dock_2016_dynamic/model.sdf.erb
  models/dock_2018_dynamic/model.sdf.erb
)

# Process the dock base erb files
foreach(_erb ${dock_base_erb_files})
  string(REGEX REPLACE ".sdf.erb" ".sdf" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     DEPENDS
                       ${CMAKE_CURRENT_SOURCE_DIR}/${_erb}
                       ${CMAKE_CURRENT_SOURCE_DIR}/dock_generator.erb
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND dock_base_files ${_model})
endforeach()
add_custom_target(dock_base_erb_generation ALL DEPENDS ${dock_base_files})

# Process the dock erb files
foreach(_erb ${dock_erb_files})
  string(REGEX REPLACE ".sdf.erb" ".sdf" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     DEPENDS
                       ${CMAKE_CURRENT_SOURCE_DIR}/${_erb}
                       ${CMAKE_CURRENT_SOURCE_DIR}/dock_generator.erb
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND dock_files ${_model})
endforeach()
add_custom_target(dock_erb_generation ALL
  DEPENDS
    ${dock_files}
    dock_base_erb_generation
)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# Generate world files from xacro and install
xacro_add_files(
  worlds/example_course.world.xacro
  worlds/navigation_task.world.xacro
  worlds/perception_task.world.xacro
  worlds/sandisland.world.xacro
  worlds/dock.world.xacro
  worlds/scan_and_dock.world.xacro
  worlds/stationkeeping_task.world.xacro
  worlds/wayfinding_task.world.xacro
  worlds/wind_test.world.xacro
  worlds/ocean.world.xacro
  ${XACRO_INORDER} INSTALL DESTINATION worlds
)
# Generate obstacle course
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_avoid_obstacles_buoys --seed 1337 --a3 6 --a5 7 --a7 7 > ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf
)
add_custom_target(${PROJECT_NAME}_generate_obstacle_course_buoys ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/robotx_2018_qualifying_avoid_obstacles_buoys/model.sdf)

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)

install(PROGRAMS scripts/spawn_wamv.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(sandisland_test
                    test/sandisland.test
                    test/sandisland.cc)
  target_link_libraries(sandisland_test ${catkin_LIBRARIES})
endif()

# Python Scripts
catkin_install_python(PROGRAMS
    nodes/twist2thrust.py
    nodes/key2thrust_angle.py
    scripts/generate_worlds
    scripts/generate_wamv
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
