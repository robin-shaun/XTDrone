#include "hybrid_astar_test/subscriber/goal_subscriber.hpp"

GoalSubscriber::GoalSubscriber(ros::NodeHandle& nh, const std::string& topic) : nh_(nh) {
    subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1, &GoalSubscriber::GoalCallback, this);
}

GoalSubscriber::~GoalSubscriber(){

}

bool GoalSubscriber::GetGoal(Eigen::Vector3f& goal){
    if(get_goal)
        goal = goal_point;
    auto flag = get_goal;
    get_goal = false;
    return flag;
}

void GoalSubscriber::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_ptr){
    global_mtx.lock();

    goal_point(0) = msg_ptr->pose.position.x;
    goal_point(1) = msg_ptr->pose.position.y;
    goal_point(2) = static_cast<float>(tf::getYaw(msg_ptr->pose.orientation));
    get_goal = true;

    global_mtx.unlock();
}