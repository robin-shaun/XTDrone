cmake_minimum_required(VERSION 2.8.3)
project(wamv_gazebo)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

find_package(catkin REQUIRED COMPONENTS
  wamv_description
  usv_gazebo_plugins
  xacro
)
catkin_package(
  CATKIN_DEPENDS wamv_description usv_gazebo_plugins xacro
)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

# Generate urdf files from xacro and install
xacro_add_files(
  urdf/wamv_gazebo.urdf.xacro
  ${XACRO_INORDER} INSTALL DESTINATION urdf
)

# Install meshes and textures
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
