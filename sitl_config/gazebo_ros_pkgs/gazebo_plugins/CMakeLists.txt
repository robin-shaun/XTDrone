cmake_minimum_required(VERSION 3.5.1)
project(gazebo_plugins)

option(ENABLE_DISPLAY_TESTS "Enable the building of tests that requires a display" OFF)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
)

# Through transitive dependencies in the packages above, gazebo_plugins depends
# on Simbody.  There is a bug in the Ubuntu Artful (17.10) version of the
# Simbody package where it includes /usr/lib/libblas.so and
# /usr/lib/liblapack.so in the CMake list of libraries even though neither of
# those two paths exist (they both really live in /usr/lib/<arch>-linux-gnu).
# We remove these two during build-time on artful below; this works because
# they both will get resolved to the proper paths during runtime linking.
find_program(LSB_RELEASE_EXEC lsb_release)
if(NOT LSB_RELEASE_EXEC STREQUAL "LSB_RELEASE_EXEC-NOTFOUND")
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -cs
    OUTPUT_VARIABLE OS_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(OS_CODENAME STREQUAL "artful")
    list(FILTER catkin_LIBRARIES EXCLUDE REGEX "/usr/lib/libblas.so")
    list(FILTER catkin_LIBRARIES EXCLUDE REGEX "/usr/lib/liblapack.so")
    list(FILTER GAZEBO_LIBRARIES EXCLUDE REGEX "/usr/lib/libblas.so")
    list(FILTER GAZEBO_LIBRARIES EXCLUDE REGEX "/usr/lib/liblapack.so")
  endif()
endif()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/CameraSynchronizer.cfg
  cfg/GazeboRosCamera.cfg
  cfg/GazeboRosOpenniKinect.cfg
  cfg/Hokuyo.cfg
  cfg/WheelSlip.cfg
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
  catkin_package(  INCLUDE_DIRS include LIBRARIES gazebo_ros_elevator)
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
  catkin_package(INCLUDE_DIRS include LIBRARIES gazebo_ros_harness)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  vision_reconfigure 
  gazebo_ros_utils 
  gazebo_ros_camera_utils 
  gazebo_ros_camera 
  gazebo_ros_triggered_camera
  gazebo_ros_multicamera 
  gazebo_ros_triggered_multicamera
  gazebo_ros_depth_camera 
  gazebo_ros_openni_kinect 
  gazebo_ros_gpu_laser 
  gazebo_ros_laser 
  gazebo_ros_block_laser 
  gazebo_ros_p3d 
  gazebo_ros_imu 
  gazebo_ros_imu_sensor
  gazebo_ros_f3d 
  gazebo_ros_ft_sensor
  gazebo_ros_bumper
  gazebo_ros_template
  gazebo_ros_projector
  gazebo_ros_prosilica
  gazebo_ros_force
  gazebo_ros_joint_state_publisher
  gazebo_ros_joint_pose_trajectory
  gazebo_ros_diff_drive
  gazebo_ros_tricycle_drive
  gazebo_ros_skid_steer_drive
  gazebo_ros_video
  gazebo_ros_planar_move
  gazebo_ros_range
  gazebo_ros_vacuum_gripper

  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  camera_info_manager
  std_msgs
)
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

## Executables
add_executable(hokuyo_node src/hokuyo_node.cpp)
add_dependencies(hokuyo_node ${PROJECT_NAME}_gencfg)
target_link_libraries(hokuyo_node
  ${catkin_LIBRARIES}
)

add_library(gazebo_ros_utils src/gazebo_ros_utils.cpp)
target_link_libraries(gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(vision_reconfigure src/vision_reconfigure.cpp)
add_dependencies(vision_reconfigure ${PROJECT_NAME}_gencfg)
target_link_libraries(vision_reconfigure ${catkin_LIBRARIES})

add_executable(camera_synchronizer src/camera_synchronizer.cpp)
add_dependencies(camera_synchronizer ${PROJECT_NAME}_gencfg)
target_link_libraries(camera_synchronizer vision_reconfigure ${catkin_LIBRARIES})

add_executable(pub_joint_trajectory_test test/pub_joint_trajectory_test.cpp)
target_link_libraries(pub_joint_trajectory_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_joint_trajectory_test ${catkin_EXPORTED_TARGETS}) # don't build until gazebo_msgs is done

add_definitions(-fPIC) # what is this for?

## Plugins
add_library(gazebo_ros_camera_utils src/gazebo_ros_camera_utils.cpp)
add_dependencies(gazebo_ros_camera_utils ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_camera_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(MultiCameraPlugin src/MultiCameraPlugin.cpp)
target_link_libraries(MultiCameraPlugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_camera src/gazebo_ros_camera.cpp)
add_dependencies(gazebo_ros_camera ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils CameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_triggered_camera src/gazebo_ros_triggered_camera.cpp)
add_dependencies(gazebo_ros_triggered_camera ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_triggered_camera gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} CameraPlugin ${catkin_LIBRARIES})


if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
  add_library(gazebo_ros_elevator src/gazebo_ros_elevator.cpp)
  add_dependencies(gazebo_ros_elevator ${PROJECT_NAME}_gencfg)
  target_link_libraries(gazebo_ros_elevator ElevatorPlugin ${catkin_LIBRARIES})
endif()

add_library(gazebo_ros_multicamera src/gazebo_ros_multicamera.cpp)
add_dependencies(gazebo_ros_multicamera ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_multicamera gazebo_ros_camera_utils MultiCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_triggered_multicamera src/gazebo_ros_triggered_multicamera.cpp)
add_dependencies(gazebo_ros_triggered_multicamera ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_triggered_multicamera gazebo_ros_camera_utils gazebo_ros_triggered_camera ${GAZEBO_LIBRARIES} MultiCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_depth_camera src/gazebo_ros_depth_camera.cpp)
add_dependencies(gazebo_ros_depth_camera ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_depth_camera gazebo_ros_camera_utils DepthCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_openni_kinect src/gazebo_ros_openni_kinect.cpp)
add_dependencies(gazebo_ros_openni_kinect ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_openni_kinect gazebo_ros_camera_utils DepthCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_gpu_laser src/gazebo_ros_gpu_laser.cpp)
target_link_libraries(gazebo_ros_gpu_laser ${catkin_LIBRARIES} GpuRayPlugin)

if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
  add_library(gazebo_ros_harness src/gazebo_ros_harness.cpp)
  add_dependencies(gazebo_ros_harness ${catkin_EXPORTED_TARGETS})
  target_link_libraries(gazebo_ros_harness
    ${Boost_LIBRARIES} HarnessPlugin ${catkin_LIBRARIES})
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 9.5)
  add_library(gazebo_ros_wheel_slip src/gazebo_ros_wheel_slip.cpp)
  add_dependencies(gazebo_ros_wheel_slip ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
  target_link_libraries(gazebo_ros_wheel_slip
    ${Boost_LIBRARIES} WheelSlipPlugin ${catkin_LIBRARIES})
endif()

add_library(gazebo_ros_laser src/gazebo_ros_laser.cpp)
target_link_libraries(gazebo_ros_laser RayPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_block_laser src/gazebo_ros_block_laser.cpp)
target_link_libraries(gazebo_ros_block_laser RayPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_p3d src/gazebo_ros_p3d.cpp)
target_link_libraries(gazebo_ros_p3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_imu src/gazebo_ros_imu.cpp)
target_link_libraries(gazebo_ros_imu ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_imu_sensor src/gazebo_ros_imu_sensor.cpp)
target_link_libraries(gazebo_ros_imu_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_f3d src/gazebo_ros_f3d.cpp)
target_link_libraries(gazebo_ros_f3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_bumper src/gazebo_ros_bumper.cpp)
add_dependencies(gazebo_ros_bumper ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_bumper ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_projector src/gazebo_ros_projector.cpp)
target_link_libraries(gazebo_ros_projector ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_prosilica src/gazebo_ros_prosilica.cpp)
add_dependencies(gazebo_ros_prosilica ${PROJECT_NAME}_gencfg)
target_link_libraries(gazebo_ros_prosilica gazebo_ros_camera_utils CameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_force src/gazebo_ros_force.cpp)
target_link_libraries(gazebo_ros_force ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_joint_state_publisher src/gazebo_ros_joint_state_publisher.cpp)
set_target_properties(gazebo_ros_joint_state_publisher PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_joint_state_publisher PROPERTIES COMPILE_FLAGS "${cxx_flags}")
add_dependencies(gazebo_ros_joint_state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_joint_state_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_joint_pose_trajectory src/gazebo_ros_joint_pose_trajectory.cpp)
add_dependencies(gazebo_ros_joint_pose_trajectory ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_joint_pose_trajectory ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_diff_drive src/gazebo_ros_diff_drive.cpp)
target_link_libraries(gazebo_ros_diff_drive gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_tricycle_drive src/gazebo_ros_tricycle_drive.cpp)
target_link_libraries(gazebo_ros_tricycle_drive gazebo_ros_utils ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_skid_steer_drive src/gazebo_ros_skid_steer_drive.cpp)
target_link_libraries(gazebo_ros_skid_steer_drive ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_video src/gazebo_ros_video.cpp)
target_link_libraries(gazebo_ros_video ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_LIBRARIES})

add_library(gazebo_ros_planar_move src/gazebo_ros_planar_move.cpp)
target_link_libraries(gazebo_ros_planar_move ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_hand_of_god src/gazebo_ros_hand_of_god.cpp)
set_target_properties(gazebo_ros_hand_of_god PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_hand_of_god PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_hand_of_god ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_ft_sensor src/gazebo_ros_ft_sensor.cpp)
target_link_libraries(gazebo_ros_ft_sensor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_range src/gazebo_ros_range.cpp)
target_link_libraries(gazebo_ros_range ${catkin_LIBRARIES} ${Boost_LIBRARIES} RayPlugin)

add_library(gazebo_ros_vacuum_gripper src/gazebo_ros_vacuum_gripper.cpp)
target_link_libraries(gazebo_ros_vacuum_gripper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##
## Add your new plugin here
##

## Template
add_library(gazebo_ros_template src/gazebo_ros_template.cpp)
target_link_libraries(gazebo_ros_template ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  hokuyo_node
  camera_synchronizer
  pub_joint_trajectory_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS
  vision_reconfigure
  gazebo_ros_utils
  gazebo_ros_camera_utils
  gazebo_ros_camera
  gazebo_ros_triggered_camera
  gazebo_ros_multicamera
  MultiCameraPlugin
  gazebo_ros_triggered_multicamera
  gazebo_ros_depth_camera
  gazebo_ros_openni_kinect
  gazebo_ros_laser
  gazebo_ros_block_laser
  gazebo_ros_p3d
  gazebo_ros_imu
  gazebo_ros_imu_sensor
  gazebo_ros_f3d
  gazebo_ros_ft_sensor
  gazebo_ros_bumper
  gazebo_ros_hand_of_god
  gazebo_ros_template
  gazebo_ros_projector
  gazebo_ros_prosilica
  gazebo_ros_force
  gazebo_ros_joint_state_publisher
  gazebo_ros_joint_pose_trajectory
  gazebo_ros_diff_drive
  gazebo_ros_tricycle_drive
  gazebo_ros_skid_steer_drive
  gazebo_ros_video
  gazebo_ros_planar_move
  gazebo_ros_vacuum_gripper
  gazebo_ros_gpu_laser
  gazebo_ros_range
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
  install(TARGETS gazebo_ros_elevator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
  install(TARGETS gazebo_ros_harness
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 9.5)
  install(TARGETS gazebo_ros_wheel_slip
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
endif()

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS scripts/set_wrench.py scripts/set_pose.py scripts/gazebo_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# Tests
# These need to be run with -j1 flag because gazebo can't be run
# in parallel.
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(set_model_state-test
                    test/set_model_state_test/set_model_state_test.test
                    test/set_model_state_test/set_model_state_test.cpp)
  target_link_libraries(set_model_state-test ${catkin_LIBRARIES})

  add_rostest(test/range/range_plugin.test)

  if (ENABLE_DISPLAY_TESTS)
    add_rostest_gtest(depth_camera-test
                      test/camera/depth_camera.test
                      test/camera/depth_camera.cpp)
    target_link_libraries(depth_camera-test ${catkin_LIBRARIES})
    add_rostest_gtest(multicamera-test
                      test/camera/multicamera.test
                      test/camera/multicamera.cpp)
    target_link_libraries(multicamera-test ${catkin_LIBRARIES})
    add_rostest_gtest(camera-test
                      test/camera/camera.test
                      test/camera/camera.cpp)
    target_link_libraries(camera-test ${catkin_LIBRARIES})
    add_rostest_gtest(camera16bit-test
                      test/camera/camera16bit.test
                      test/camera/camera16bit.cpp)
    target_link_libraries(camera16bit-test ${catkin_LIBRARIES})
    add_rostest_gtest(distortion_barrel_test
                      test/camera/distortion_barrel.test
                      test/camera/distortion_barrel.cpp)
    target_link_libraries(distortion_barrel_test ${catkin_LIBRARIES})
    add_rostest_gtest(distortion_pincushion_test
                      test/camera/distortion_pincushion.test
                      test/camera/distortion_pincushion.cpp)
    target_link_libraries(distortion_pincushion_test ${catkin_LIBRARIES})
    add_rostest_gtest(triggered-camera-test
                      test/camera/triggered_camera.test
                      test/camera/triggered_camera.cpp)
    target_link_libraries(triggered-camera-test ${catkin_LIBRARIES})
  endif()
endif()
