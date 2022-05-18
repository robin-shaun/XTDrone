cmake_minimum_required(VERSION 3.5.1)
project(gazebo_ros)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  cmake_modules
  roslib
  roscpp
  geometry_msgs
  std_srvs
  tf
  rosgraph_msgs
  dynamic_reconfigure
  std_msgs
  gazebo_msgs
)

# Through transitive dependencies in the packages above, gazebo_ros depends
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
  endif()
endif()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

find_package(TinyXML REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/Physics.cfg)

catkin_package(
  LIBRARIES
    gazebo_ros_api_plugin
    gazebo_ros_paths_plugin

  CATKIN_DEPENDS
    roslib
    roscpp
    geometry_msgs
    std_srvs
    tf
    rosgraph_msgs
    dynamic_reconfigure
    std_msgs
    gazebo_msgs

  DEPENDS
    TinyXML
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()

## Plugins
add_library(gazebo_ros_api_plugin src/gazebo_ros_api_plugin.cpp)
add_dependencies(gazebo_ros_api_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(gazebo_ros_api_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_api_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_api_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})

add_library(gazebo_ros_paths_plugin src/gazebo_ros_paths_plugin.cpp)
add_dependencies(gazebo_ros_paths_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(gazebo_ros_paths_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
set_target_properties(gazebo_ros_paths_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
target_link_libraries(gazebo_ros_paths_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Tests

add_subdirectory(test)

# Install Gazebo System Plugins
install(TARGETS gazebo_ros_api_plugin gazebo_ros_paths_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

# Install Gazebo Scripts
if (WIN32)
  install(PROGRAMS
                  scripts/debug.bat
                  scripts/gzclient.bat
                  scripts/gzserver.bat
          DESTINATION
                  ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else ()
  install(PROGRAMS
                  scripts/gazebo
                  scripts/debug
                  scripts/gzclient
                  scripts/gzserver
                  scripts/gdbrun
                  scripts/perf
                  scripts/libcommon.sh
          DESTINATION
                  ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# This one is a Python program, not a shell script, so install it separately
catkin_install_python(PROGRAMS scripts/spawn_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Gazebo launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
