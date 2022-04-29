#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_GOAL_SUBSCRIBER_H_PP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_GOAL_SUBSCRIBER_H_PP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <mutex>

class GoalSubscriber {
public:
    GoalSubscriber(ros::NodeHandle& nh, const std::string& topic);
    ~GoalSubscriber();

public:
    bool GetGoal(Eigen::Vector3f& goal);

private:
    void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_ptr);

private:

    ros::Subscriber subscriber_;
    ros::NodeHandle nh_;

    Eigen::Vector3f goal_point;
    bool            get_goal = false;

    std::mutex      global_mtx;
};

#endif