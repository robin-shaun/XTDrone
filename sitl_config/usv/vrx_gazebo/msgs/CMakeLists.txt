find_package(Protobuf REQUIRED)

# To remove policy warnings in CMake 3.10. See `cmake --help-policy CMP0071`
if(POLICY CMP0071)
  # Set to NEW to let AUTOMOC and AUTOUIC process generated files
  cmake_policy(SET CMP0071 NEW)
endif()

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-11.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_PROTO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(msgs1
  light_buoy_colors.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS1 PROTO_HDRS ${msgs1})

add_library(light_buoy_colors_msgs SHARED ${PROTO_SRCS1})
target_link_libraries(light_buoy_colors_msgs 
  ${PROTOBUF_LIBRARY}
  ${GAZEBO_LIBRARIES}
  ${GAZEBO_PROTO_LIBRARIES}
)

set(msgs2
  dock_placard.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS2 PROTO_HDRS ${msgs2})

add_library(dock_placard_msgs SHARED ${PROTO_SRCS2})

target_link_libraries(dock_placard_msgs
  ${PROTOBUF_LIBRARY} 
  ${GAZEBO_LIBRARIES}
  ${GAZEBO_PROTO_LIBRARIES}
)

install(TARGETS dock_placard_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS light_buoy_colors_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
