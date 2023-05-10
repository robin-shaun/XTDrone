find_package(Protobuf REQUIRED)

set(msgs
 grasp_event.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gazebo_grasp_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gazebo_grasp_msgs ${PROTOBUF_LIBRARY})

## Mark executables and/or libraries for installation
install(TARGETS gazebo_grasp_msgs
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.h"
)
