#ifndef HYBRID_ASTAR_TEST_NAVIGATION_NAVIGATION_PURE_PURSUIT_HPP
#define HYBRID_ASTAR_TEST_NAVIGATION_NAVIGATION_PURE_PURSUIT_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <memory>

#include "hybrid_astar_test/subscriber/start_subscriber.hpp"
#include "hybrid_astar_test/subscriber/path_subscriber.hpp"

class PurePursuitControl{
public:
    PurePursuitControl(ros::NodeHandle& nh);
    ~PurePursuitControl();

public:
    bool Run();

private:
    Eigen::Vector2f CalculationVelocity(const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose, float delta_time);
    Eigen::Vector2f CalculationAcceleration(const Eigen::Vector2f& last_v, const Eigen::Vector2f& now_v, const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose);



private:
    std::shared_ptr<StartSubscriber>     localization_subscriber_ptr_;
    std::shared_ptr<PathSubscriber>      path_subscriber_ptr_;

private:
    ros::NodeHandle nh_;

    std::mutex global_mtx_;
    Eigen::VectorXf start_point(6);
    Eigen::VectorXf robot_localization(7);  // x y yaw liner_v angle_v liner_acc angle_acc
    std::vector<Eigen::Vector3f> Trajectory;
};

#endif