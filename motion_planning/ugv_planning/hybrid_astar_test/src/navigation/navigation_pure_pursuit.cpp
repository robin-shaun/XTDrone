#include "hybrid_astar_test/navigation/navigation_pure_pursuit.hpp"

PurePursuitControl::PurePursuitControl(ros::NodeHandle& nh) : nh_(nh){

    localization_subscriber_ptr_ = std::make_shared<StartSubscriber>(nh_, "catvehicle/base_link", "/map");
    path_subscriber_ptr_ = std::make_shared<PathSubscriber>(nh_, "/front_end__poses");

}

bool PurePursuitControl::Run(){

    
    static Eigen::Vector3f start_point;
    if(!localization_subscriber_ptr_->GetStart(start_point))
        return false;

    
    static std::vector<Eigen::Vector3f> trajectory;
    if(!path_subscriber_ptr_->GetPath(trajectory))
        return false;


}

Eigen::Vector2f PurePursuitControl::CalculationVelocity(const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose, float delta_time){
    float liner_v = (now_pose.block<2,1>(0,0) - last_pose.block<2,1>(0,0)).norm() / delta_time;
    float angle_v = calculation_yaw_difference(now_pose(2), last_pose(2)) / delta_time;
    if(abs(liner_v) < 0.001)
        liner_v = 0.0;
    if(abs(angle_v) < 0.001)
        angle_v = 0.0;
    return Eigen::Vector2f(liner_v, angle_v);
}

Eigen::Vector2f PurePursuitControl::CalculationAcceleration(const Eigen::Vector2f& last_v, const Eigen::Vector2f& now_v, const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose){
    float x = (now_pose.block<2,1>(0,0) - last_pose.block<2,1>(0,0)).norm();
    float yaw = calculation_yaw_difference(now_pose(2), last_pose(2));
    float acc_x = (now_v(0) * now_v(0) - last_v(0) * last_v(0)) / (2.0 * x);
    float acc_a = (now_v(1) * now_v(1) - last_v(1) * last_v(1)) / (2.0 * yaw);
    if(abs(x) < 0.001)
        acc_x = 0.0;
    if(abs(yaw) < 0.001)
        acc_a = 0.0;
    return Eigen::Vector2f(acc_x, acc_a);
}

