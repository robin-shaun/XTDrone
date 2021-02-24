cmake_minimum_required(VERSION 2.8.3)
project(wamv_description)
find_package(catkin REQUIRED COMPONENTS xacro)
catkin_package(CATKIN_DEPENDS xacro)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

xacro_add_files(
  urdf/wamv_base.urdf.xacro
    ${XACRO_INORDER} INSTALL DESTINATION urdf
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
