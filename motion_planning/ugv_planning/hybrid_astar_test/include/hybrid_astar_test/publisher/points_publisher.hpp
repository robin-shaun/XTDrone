#ifndef HYBRID_ASTAR_TEST_PUBLISHER_POINTS_PUBLISHER_H_PP
#define HYBRID_ASTAR_TEST_PUBLISHER_POINTS_PUBLISHER_H_PP

#include <ros/ros.h>
#include <Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class PointsPublisher {
public:
    PointsPublisher(ros::NodeHandle& nh, const std::string& topic);
    ~PointsPublisher(){};

public:
    void Publish(const Eigen::Vector3f& point);

private:
    void PublishVectorData(const Eigen::Vector3f& point);
    void PublishPointData(const Eigen::Vector3f& point);

private:
    ros::Publisher  vector_publisher_;
    ros::Publisher  poses_publisher_;
    ros::NodeHandle nh_;
};

#endif