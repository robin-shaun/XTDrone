#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_EST_START_SUBSCRIBER_H_PP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_EST_START_SUBSCRIBER_H_PP

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <mutex>

class EstStartSubscriber {
public:
    EstStartSubscriber(ros::NodeHandle& nh, const std::string& topic);
    ~EstStartSubscriber();

public:
    bool GetStart(Eigen::Vector3f& start);

private:
    void StartCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_ptr);

private:

    ros::Subscriber subscriber_;
    ros::NodeHandle nh_;

    Eigen::Vector3f start_point;
    bool            get_start = false;

    std::mutex      global_mtx;
};

#endif