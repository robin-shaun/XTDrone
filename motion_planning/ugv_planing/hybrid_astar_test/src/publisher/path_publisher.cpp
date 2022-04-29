#include "hybrid_astar_test/publisher/path_publisher.hpp"

PathPublisher::PathPublisher(ros::NodeHandle& nh, const std::string& topic) : nh_(nh){
    path_publisher_  = nh_.advertise<nav_msgs::Path>(topic + "_path", 1);
    poses_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(topic + "_poses", 1);
}

void PathPublisher::Publish(const std::vector<Eigen::Vector3f>& path){

    PublishPathData(path);
    PublishPoseData(path);
}

void PathPublisher::PublishPathData(const std::vector<Eigen::Vector3f>& path){
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    for(int i = 0 ; i < path.size() - 1; i++){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = path[i].x();
        pose.pose.position.y = path[i].y();
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        path_msg.poses.push_back(pose);
    }
    path_publisher_.publish(path_msg);
}

void PathPublisher::PublishPoseData(const std::vector<Eigen::Vector3f>& path){
    geometry_msgs::PoseArray poses_msg;
    poses_msg.header.frame_id = "map";
    poses_msg.header.stamp = ros::Time::now();
    float history_yaw = 0.0;
    for(int i = 0 ; i < path.size() -1; i++){
        geometry_msgs::Pose pose;
        pose.position.x = path[i].x();
        pose.position.y = path[i].y();
        pose.position.z = 0;
        pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        poses_msg.poses.push_back(pose);
    }
    poses_publisher_.publish(poses_msg);
}