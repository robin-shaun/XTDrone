#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_START_SUBSCRIBER_H_PP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_START_SUBSCRIBER_H_PP

#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>
#include <mutex>
#include <boost/bind.hpp>

#include <Eigen/Dense>

class StartSubscriber {
public:
    StartSubscriber(ros::NodeHandle& nh, const std::string& base_frame, const std::string& world_frame);
    ~StartSubscriber();

public:
    bool GetStart(Eigen::Vector3f& start);

private:
    void StartCallback();

private:

    ros::Subscriber clear_costmap_sub_;
    ros::NodeHandle nh_;
    tf2_ros::Buffer *buffer;
    tf2_ros::TransformListener *tf;
    tf::TransformListener* listener;

    Eigen::Vector3f start_point;
    bool            get_start = false;

    std::string base_frame_,world_frame_;

    std::mutex      global_mtx;
    bool            end_flag = false;
};

#endif