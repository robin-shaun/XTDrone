#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_COSTMAP_SUBSCRIBER_H_PP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_COSTMAP_SUBSCRIBER_H_PP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <thread>
#include <mutex>

class CostmapSubscriber {
public:
    CostmapSubscriber(ros::NodeHandle& nh, const std::string& topic);
    ~CostmapSubscriber();

public:
    bool GetCostMap(nav_msgs::OccupancyGrid& map, float& map_origin_x, float& map_origin_y);

private:
    void CostMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

private:
    ros::Subscriber costmap_subscriber_ptr_;
    ros::NodeHandle nh_;
    std::mutex      global_mtx_;
    bool            get_map = false;

    nav_msgs::OccupancyGrid costmap_;
};

#endif