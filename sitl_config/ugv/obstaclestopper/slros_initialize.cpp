#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "obstacleStopper";

// For Block obstacleStopper/Subscribe
SimulinkSubscriber<std_msgs::Float64, SL_Bus_obstacleStopper_std_msgs_Float64> Sub_obstacleStopper_12;

// For Block obstacleStopper/Subscribe1
SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Sub_obstacleStopper_13;

// For Block obstacleStopper/Subscribe2
SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Sub_obstacleStopper_39;

// For Block obstacleStopper/Publish
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Pub_obstacleStopper_17;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

