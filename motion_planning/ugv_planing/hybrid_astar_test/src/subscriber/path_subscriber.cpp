#include "hybrid_astar_test/subscriber/path_subscriber.hpp"

PathSubscriber::PathSubscriber(ros::NodeHandle& nh, const std::string& topic){

    subscriber_ = nh_.subscribe<geometry_msgs::PoseArray>(topic.c_str(), 1, &PathSubscriber::PathCallback, this);

}

bool PathSubscriber::GetPath(std::vector<Eigen::Vector3f>& path){
    if(get_trajectory_flag_)
        path = trajectory_;
    auto flag = get_trajectory_flag_;
    get_trajectory_flag_ = false;
    return flag;
}

void PathSubscriber::PathCallback(const geometry_msgs::PoseArray::ConstPtr& msg_ptr){
    global_mtx_.lock();
    trajectory_.clear();

    for(int i = 0; i < msg_ptr->poses.size(); i++){
        trajectory_.push_back(Eigen::Vector3f(msg_ptr->poses[i].position.x,
                                              msg_ptr->poses[i].position.y,
                                              tf::getYaw(msg_ptr->poses[i].orientation))));
    }

    get_trajectory_flag_ = true;

    global_mtx_.unlock();

}