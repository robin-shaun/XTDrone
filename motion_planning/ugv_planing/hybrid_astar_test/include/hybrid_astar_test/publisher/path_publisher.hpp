#ifndef HYBRID_ASTAR_TEST_PUBLISHER_PATH_PUBLISHER_H_PP
#define HYBRID_ASTAR_TEST_PUBLISHER_PATH_PUBLISHER_H_PP

#include <ros/ros.h>
#include <Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class PathPublisher {
public:
    PathPublisher(ros::NodeHandle& nh, const std::string& topic);
    ~PathPublisher(){};

public:
    void Publish(const std::vector<Eigen::Vector3f>& path);

private:
    void PublishPathData(const std::vector<Eigen::Vector3f>& path);
    void PublishPoseData(const std::vector<Eigen::Vector3f>& path);

private:
    ros::Publisher  path_publisher_;
    ros::Publisher  poses_publisher_;
    ros::NodeHandle nh_;
};

#endif