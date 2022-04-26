#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_PATH_SUBSCRIBER_H_PP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_PATH_SUBSCRIBER_H_PP

#include <ros/ros.h>
#include <Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <mutex>

class PathSubscriber {
public:
    PathSubscriber(ros::NodeHandle& nh, const std::string& topic);
    ~PathSubscriber(){};

public:
    bool GetPath(std::vector<Eigen::Vector3f>& path);

private:
    void PathCallback(const geometry_msgs::PoseArray::ConstPtr& msg_ptr);

private:
    ros::Subscriber     subscriber_;
    ros::NodeHandle     nh_;
    std::mutex          global_mtx_;
    bool                get_trajectory_flag_ = false;
    std::vector<Eigen::Vector3f> trajectory_;
};

#endif