#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_MAP_SUBSCRIBER_H_PP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_MAP_SUBSCRIBER_H_PP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>
#include <boost/bind.hpp>
#include <std_msgs/Bool.h>

class MapSubscriber {
public:
    MapSubscriber(ros::NodeHandle& nh, const std::string& map_type);
    ~MapSubscriber();

public:
    void GetCostMap();

private:
    void ClearCostMap(const std_msgs::Bool::ConstPtr &msg);

    void CostMapThread();

private:
    costmap_2d::Costmap2DROS *costmap_ros_;
    ros::Subscriber clear_costmap_sub_;
    ros::NodeHandle nh_;
    tf2_ros::Buffer *buffer;
    tf2_ros::TransformListener *tf;
    std::string map_type_;
};

#endif