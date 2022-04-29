cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_test)
set(CMAKE_BUILD_TYPE "Release")

set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file(
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/global_definition/global_definition.h.in
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/global_definition/global_definition.h
)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/apps/main.cpp
                                    src/navigation/path_search.cpp
                                    src/subscriber/start_subscriber.cpp
                                    src/subscriber/est_start_subscriber.cpp
                                    src/subscriber/goal_subscriber.cpp
                                    src/subscriber/map_subscriber.cpp
                                    src/subscriber/costmap_subscriber.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/collision_detection.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/dubins.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/node3d.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/node2d.cpp
                                    src/global_definition/constants.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/algorithm.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/visualize.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/path.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/bucketedqueue.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/dynamic_voronoi.cpp
                                    src/algorithm/path_searcher/hybrid_a_star/smoother.cpp
                                    src/publisher/path_publisher.cpp
                                    src/publisher/tf_broadcaster.cpp
                                    src/publisher/points_publisher.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${OMPL_LIBRARIES}
)
