

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

if (NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "install prefix" FORCE)
endif()

message(STATUS "install-prefix: ${CMAKE_INSTALL_PREFIX}")

project(mavlink_sitl_gazebo VERSION 1.0.0)
include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(BUILD_GSTREAMER_PLUGIN "enable gstreamer plugin" ON)
option(BUILD_ROS_INTERFACE "Enable building ROS dependent plugins" OFF)

option(SEND_VISION_ESTIMATION_DATA "Send Mavlink VISION_POSITION_ESTIMATE msgs" OFF)
option(SEND_ODOMETRY_DATA "Send Mavlink ODOMETRY msgs" OFF)

# Set c++11 or higher
include(EnableC++XX)

## System dependencies are found with CMake's conventions
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
find_program(px4 REQUIRED)
# Note: If using catkin, Python 2 is found since it points
# to the Python libs installed with the ROS distro
if (NOT CATKIN_DEVEL_PREFIX)
	find_package(PythonInterp 3 REQUIRED)
else()
	find_package(PythonInterp REQUIRED)
endif()
find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)
if (BUILD_GSTREAMER_PLUGIN)
  set(GStreamer_FIND_VERSION "1.0")
  find_package(GStreamer REQUIRED)
  if (GSTREAMER_FOUND)
    if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
      find_package (Qt4)
      include (${QT_USE_FILE})
    else()
      # In order to find Qt5 in macOS, the Qt5 path needs to be added to the CMake prefix path.
      if(APPLE)
        execute_process(COMMAND brew --prefix qt5
                        ERROR_QUIET
                        OUTPUT_VARIABLE QT5_PREFIX_PATH
                        OUTPUT_STRIP_TRAILING_WHITESPACE
                       )
        list(APPEND CMAKE_PREFIX_PATH "${QT5_PREFIX_PATH}/lib/cmake")
      endif()
      find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
    endif()
  endif()
endif()

pkg_check_modules(OGRE OGRE)

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
else()
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})

add_subdirectory( external/OpticalFlow OpticalFlow )
set( OpticalFlow_LIBS "OpticalFlow" )

# for ROS subscribers and publishers
if (BUILD_ROS_INTERFACE)
  find_package(geometry_msgs REQUIRED)
  find_package(roscpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
endif()

# find MAVLink
find_package(MAVLink)

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
  message(STATUS "catkin ENABLED")
  find_package(catkin REQUIRED)
  if (catkin_FOUND)
    catkin_package()
  else()
    message(FATAL_ERROR "catkin not found")
  endif()
else()
  message(STATUS "catkin DISABLED")
endif()

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###########
## Build ##
###########

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wno-deprecated-declarations -Wno-address-of-packed-member")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-address-of-packed-member")

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
  ${OpenCV_INCLUDE_DIRS}
  ${OpticalFlow_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  )

if (GSTREAMER_FOUND)
  include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
  )
endif()

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  )

if (GSTREAMER_FOUND)
  link_libraries(
    ${GSTREAMER_LIBRARIES}
    glib-2.0
    gobject-2.0
  )
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_LIBRARY_DIRS}
  )

#--------------------------#
# Generation of SDF models #
#--------------------------#

set(enable_mavlink_interface "true")
set(enable_ground_truth "false")
set(enable_logging "false")
set(enable_camera "false")
set(enable_wind "false")
set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/rotors_description")
set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
# set the vision estimation to be sent if set by the CMake option SEND_VISION_ESTIMATION_DATA
set(send_vision_estimation "false")
if (SEND_VISION_ESTIMATION_DATA)
  set(send_vision_estimation "true")
endif()

# if SEND_ODOMETRY_DATA option is set, then full odometry data is sent instead of
# only the visual pose estimate
set(send_odometry "false")
if (SEND_ODOMETRY_DATA)
  set(send_odometry "true")
  set(send_vision_estimation "false")
endif()

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
  COMMAND ${PYTHON_EXECUTABLE} ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir} send_vision_estimation:=${send_vision_estimation} send_odometry:=${send_odometry}
  COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
  COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
  DEPENDS ${rotors_description_dir}/urdf/iris.xacro
  DEPENDS ${rotors_description_dir}/urdf/iris_base.xacro
  DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
  )
add_custom_target(sdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf)


#-----------#
# Functions #
#-----------#

function(glob_generate target file_glob)
  file(GLOB_RECURSE glob_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${file_glob})
	set(gen_files)
	foreach(glob_file ${glob_files})
		string(REGEX REPLACE "\\.[^.]*$" "" file_name ${glob_file})
		string(REGEX MATCH "[^.]*$" file_ext ${glob_file})
		get_filename_component(file_dir ${glob_file} DIRECTORY)
		set(in_file ${CMAKE_CURRENT_SOURCE_DIR}/${glob_file})
		file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${file_dir})
		set(out_file ${CMAKE_CURRENT_SOURCE_DIR}/${file_name})
		string(REGEX REPLACE ".sdf" "-gen.sdf" out_file ${out_file})
		if (${file_ext} STREQUAL "jinja")
			add_custom_command(OUTPUT ${out_file}
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/jinja_gen.py
        ${in_file} ${CMAKE_CURRENT_SOURCE_DIR}
				DEPENDS ${in_file}
				VERBATIM
				)
			list(APPEND gen_files_${target} ${out_file})
		endif()
	endforeach()
	add_custom_target(${target} ALL DEPENDS ${gen_files_${target}})
endfunction()

glob_generate(models_gen ${CMAKE_CURRENT_SOURCE_DIR}/models/*.jinja)

#--------------------#
# Message Generation #
#--------------------#

set(mav_msgs
  msgs/CommandMotorSpeed.proto
  msgs/MotorSpeed.proto
  msgs/Actor.proto
  msgs/CmdActor.proto
  )
set(nav_msgs msgs/Odometry.proto)
set(physics_msgs msgs/Wind.proto)
set(std_msgs msgs/Int32.proto)
set(sensor_msgs
  msgs/Imu.proto
  msgs/IRLock.proto
  msgs/Float.proto
  msgs/Groundtruth.proto
  msgs/Range.proto
  msgs/SITLGps.proto
  msgs/OpticalFlow.proto
  msgs/MagneticField.proto
  msgs/Pressure.proto
  )

PROTOBUF_GENERATE_CPP(MAV_PROTO_SRCS MAV_PROTO_HDRS ${mav_msgs})
PROTOBUF_GENERATE_CPP(NAV_PROTO_SRCS NAV_PROTO_HDRS ${nav_msgs})
PROTOBUF_GENERATE_CPP(PHY_PROTO_SRCS PHY_PROTO_HDRS ${physics_msgs})
PROTOBUF_GENERATE_CPP(STD_PROTO_SRCS STD_PROTO_HDRS ${std_msgs})
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

add_library(mav_msgs SHARED ${MAV_PROTO_SRCS})
add_library(nav_msgs SHARED ${NAV_PROTO_SRCS})
add_library(physics_msgs SHARED ${PHY_PROTO_SRCS})
add_library(std_msgs SHARED ${STD_PROTO_SRCS})
add_library(sensor_msgs SHARED ${SEN_PROTO_SRCS})

#---------#
# Plugins #
#---------#

link_libraries(mav_msgs nav_msgs std_msgs sensor_msgs)
link_libraries(physics_msgs)

add_library(gazebo_geotagged_images_plugin SHARED src/gazebo_geotagged_images_plugin.cpp)
add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
add_library(gazebo_irlock_plugin SHARED src/gazebo_irlock_plugin.cpp)
add_library(gazebo_lidar_plugin SHARED src/gazebo_lidar_plugin.cpp)
add_library(gazebo_opticalflow_mockup_plugin SHARED src/gazebo_opticalflow_mockup_plugin.cpp)
add_library(gazebo_opticalflow_plugin SHARED src/gazebo_opticalflow_plugin.cpp)
add_library(gazebo_sonar_plugin SHARED src/gazebo_sonar_plugin.cpp)
add_library(gazebo_uuv_plugin SHARED src/gazebo_uuv_plugin.cpp)
add_library(gazebo_vision_plugin SHARED src/gazebo_vision_plugin.cpp)
add_library(gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
add_library(gazebo_gimbal_controller_plugin SHARED src/gazebo_gimbal_controller_plugin.cpp)
add_library(gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
add_library(gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp )
add_library(gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
add_library(gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
add_library(gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)
add_library(gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp src/geo_mag_declination.cpp)
add_library(gazebo_barometer_plugin SHARED src/gazebo_barometer_plugin.cpp)
add_library(gazebo_catapult_plugin SHARED src/gazebo_catapult_plugin.cpp)
add_library(gazebo_usv_dynamics_plugin SHARED src/gazebo_usv_dynamics_plugin.cpp)
add_library(gazebo_parachute_plugin SHARED src/gazebo_parachute_plugin.cpp)
add_library(ActorPlugin1 SHARED src/ActorPlugin1.cpp)
add_library(ActorPlugin2 SHARED src/ActorPlugin2.cpp)
add_library(ActorPlugin3 SHARED src/ActorPlugin3.cpp)
add_library(ActorPlugin4 SHARED src/ActorPlugin4.cpp)
add_library(ActorPlugin5 SHARED src/ActorPlugin5.cpp)

set(plugins
  gazebo_geotagged_images_plugin
  gazebo_gps_plugin
  gazebo_irlock_plugin
  gazebo_lidar_plugin
  gazebo_opticalflow_mockup_plugin
  gazebo_opticalflow_plugin
  gazebo_sonar_plugin
  gazebo_uuv_plugin
  gazebo_vision_plugin
  gazebo_controller_interface
  gazebo_gimbal_controller_plugin
  gazebo_imu_plugin
  gazebo_mavlink_interface
  gazebo_motor_model
  gazebo_multirotor_base_plugin
  gazebo_wind_plugin
  gazebo_magnetometer_plugin
  gazebo_barometer_plugin
  gazebo_catapult_plugin
  gazebo_usv_dynamics_plugin
  gazebo_parachute_plugin
  ActorPlugin1
  ActorPlugin2
  ActorPlugin3
  ActorPlugin4
  ActorPlugin5
  )

foreach(plugin ${plugins})
  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${TinyXML_LIBRARIES})
endforeach()
target_link_libraries(gazebo_opticalflow_plugin ${OpticalFlow_LIBS})

# If BUILD_ROS_INTERFACE set to ON, build plugins that have ROS dependencies
# Current plugins that can be used with ROS interface: gazebo_motor_failure_plugin
if (BUILD_ROS_INTERFACE)
  add_library(gazebo_motor_failure_plugin SHARED src/gazebo_motor_failure_plugin.cpp)
  target_link_libraries(gazebo_motor_failure_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
  list(APPEND plugins gazebo_motor_failure_plugin)
  message(STATUS "adding gazebo_motor_failure_plugin to build")

  include_directories(
    include
    ${geometry_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
  )

  target_link_libraries(gazebo_motor_failure_plugin
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${GAZEBO_libraries}
  )
endif()

if (GSTREAMER_FOUND)
  add_library(gazebo_gst_camera_plugin SHARED src/gazebo_gst_camera_plugin.cpp)
  set(plugins
    ${plugins}
    gazebo_gst_camera_plugin
  )
  message(STATUS "Found GStreamer: adding gst_camera_plugin")
  if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
    QT4_WRAP_CPP(headers_MOC include/gazebo_video_stream_widget.h)
    add_library(gazebo_video_stream_widget SHARED ${headers_MOC} src/gazebo_video_stream_widget.cpp)
    target_link_libraries(gazebo_video_stream_widget ${GAZEBO_LIBRARIES} ${QT_LIBRARIES} ${PROTOBUF_LIBRARIES})
    set(plugins
      ${plugins}
      gazebo_video_stream_widget
    )
    message(STATUS "Found GStreamer: adding gst_video_stream_widget")
  else()
    QT5_WRAP_CPP(headers_MOC include/gazebo_video_stream_widget.h)
    add_library(gazebo_video_stream_widget SHARED ${headers_MOC} src/gazebo_video_stream_widget.cpp)
    target_link_libraries(gazebo_video_stream_widget ${GAZEBO_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Qt5Test_LIBRARIES})
    set(plugins
      ${plugins}
      gazebo_video_stream_widget
    )
    message(STATUS "Found GStreamer: adding gst_video_stream_widget")
  endif()
endif()

# Linux is not consistent with plugin availability, even on Gazebo 7
#if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
  add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
  list(APPEND plugins LiftDragPlugin)
#endif()

foreach(plugin ${plugins})
  add_dependencies(${plugin} mav_msgs nav_msgs std_msgs sensor_msgs)
  add_dependencies(${plugin} physics_msgs)
endforeach()

# Configure the setup script
if (catkin_FOUND)
  catkin_add_env_hooks(50_sitl_gazebo_setup
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    SHELLS sh)
endif()


################
## Unit Tests ##
################
include(UnitTests)

add_subdirectory(unit_tests)


#############
## Install ##
#############

set(PLUGIN_PATH ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins)
set(MODEL_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models)
set(RESOURCE_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/models/.DS_Store)
file(GLOB models_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/models/*)

file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/worlds/.DS_Store)
file(GLOB worlds_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/worlds/*)

install(TARGETS ${plugins} mav_msgs nav_msgs std_msgs sensor_msgs DESTINATION ${PLUGIN_PATH})
install(DIRECTORY ${models_list} DESTINATION ${MODEL_PATH})
install(FILES ${worlds_list} DESTINATION ${RESOURCE_PATH}/worlds)

configure_file(src/setup.sh.in "${CMAKE_CURRENT_BINARY_DIR}/setup.sh" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${RESOURCE_PATH})

install(FILES ${PROJECT_SOURCE_DIR}/package.xml DESTINATION ${RESOURCE_PATH})

#############
## Testing ##
#############

# TODO

###############
## Packaging ##
###############

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT pxusers@googlegroups.com)
set(DEBIAN_PACKAGE_DEPENDS "")
set(RPM_PACKAGE_DEPENDS "")

set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_PACKAGE_DEPENDS})
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "gazebo plugins for px4 sitl.")

set(CPACK_RPM_PACKAGE_REQUIRES "${DEBIAN_PACKAGE_DEPENDS}")
set(CPACK_RPM_PACKAGE_DESCRIPTION "Gazebo plugins for px4 sitl.")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")

include(CPack)


