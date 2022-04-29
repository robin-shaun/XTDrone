#include "hybrid_astar_test/publisher/points_publisher.hpp"

PointsPublisher::PointsPublisher(ros::NodeHandle& nh, const std::string& topic) : nh_(nh){
    poses_publisher_  = nh_.advertise<geometry_msgs::PointStamped>(topic + "_point", 1);
    vector_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic + "_vector", 1);
}

void PointsPublisher::Publish(const Eigen::Vector3f& point){

    PublishVectorData(point);
    PublishPointData(point);

}

void PointsPublisher::PublishVectorData(const Eigen::Vector3f& point){

    geometry_msgs::PoseStamped vector_msg;
    vector_msg.header.frame_id = "map";
    vector_msg.header.stamp = ros::Time::now();

    vector_msg.pose.position.x = point.x();
    vector_msg.pose.position.y = point.y();
    vector_msg.pose.position.z = 0;
    vector_msg.pose.orientation = tf::createQuaternionMsgFromYaw(point.z());
    vector_publisher_.publish(vector_msg);
}

void PointsPublisher::PublishPointData(const Eigen::Vector3f& point){

    geometry_msgs::PointStamped points_msg;
    points_msg.header.frame_id = "map";
    points_msg.header.stamp = ros::Time::now();
    points_msg.point.x = point.x();
    points_msg.point.y = point.y();
    points_msg.point.z = 0;

    poses_publisher_.publish(points_msg);
}