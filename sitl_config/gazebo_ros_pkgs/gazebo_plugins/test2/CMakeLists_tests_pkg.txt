cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_executable(spawn_box test/spawn_model/spawn_box.cpp)
rosbuild_add_gtest_build_flags(spawn_box)
rosbuild_add_rostest_labeled(gazebo test/spawn_model/spawn_box.launch)

rosbuild_add_executable(check_model test/spawn_model/check_model.cpp)
rosbuild_add_gtest_build_flags(check_model)
rosbuild_add_rostest_labeled(gazebo test/spawn_model/spawn_box_file.launch)
rosbuild_add_rostest_labeled(gazebo test/spawn_model/spawn_box_param.launch)

rosbuild_add_executable(contact_tolerance test/contact_tolerance/contact_tolerance.cpp)
rosbuild_add_gtest_build_flags(contact_tolerance)
rosbuild_add_rostest_labeled(gazebo test/contact_tolerance/contact_tolerance.launch)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
