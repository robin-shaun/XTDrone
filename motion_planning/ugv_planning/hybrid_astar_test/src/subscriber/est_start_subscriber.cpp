#include "hybrid_astar_test/subscriber/est_start_subscriber.hpp"

EstStartSubscriber::EstStartSubscriber(ros::NodeHandle& nh, const std::string& topic) : nh_(nh) {
    subscriber_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topic, 1, &EstStartSubscriber::StartCallback, this);
}

EstStartSubscriber::~EstStartSubscriber(){

}

bool EstStartSubscriber::GetStart(Eigen::Vector3f& start){
    start = start_point;
    auto flag = get_start;
    get_start = false;
    return flag;
}

void EstStartSubscriber::StartCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_ptr){
    global_mtx.lock();

    start_point(0) = msg_ptr->pose.pose.position.x;
    start_point(1) = msg_ptr->pose.pose.position.y;
    start_point(2) = static_cast<float>(tf::getYaw(msg_ptr->pose.pose.orientation));
    get_start = true;

    global_mtx.unlock();
}