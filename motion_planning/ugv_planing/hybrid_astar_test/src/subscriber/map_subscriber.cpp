#include "hybrid_astar_test/subscriber/map_subscriber.hpp"

MapSubscriber::MapSubscriber(ros::NodeHandle &nh, const std::string &map_type) : nh_(nh), map_type_(map_type) {

    clear_costmap_sub_ = nh_.subscribe<std_msgs::Bool>("clear_costmap", 1000, &MapSubscriber::ClearCostMap, this);

    std::thread costmap_thread(boost::bind(&MapSubscriber::CostMapThread, this) );
    costmap_thread.detach();
}

MapSubscriber::~MapSubscriber(){
    delete buffer;
    delete tf;
    delete costmap_ros_;
}

void MapSubscriber::GetCostMap(){

}

void MapSubscriber::ClearCostMap(const std_msgs::Bool::ConstPtr &msg){
    // 清除历史代价层
    if(msg->data == true){
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(costmap_ros_->getCostmap()->getMutex()));
        costmap_ros_->resetLayers();
    }
}

void MapSubscriber::CostMapThread(){

    buffer = new tf2_ros::Buffer(ros::Duration(10));
    tf     = new tf2_ros::TransformListener(*buffer);

    costmap_ros_ = new costmap_2d::Costmap2DROS(map_type_.c_str(), *buffer);

    costmap_ros_->pause();
    try {
        costmap_ros_->start();
    } catch (...) {
        costmap_ros_->stop();
    }

}
