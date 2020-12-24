#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block cmdvel2gazebo/Subscribe
extern SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_cmdvel2gazebo_geometry_msgs_Twist> Sub_cmdvel2gazebo_3;

// For Block cmdvel2gazebo/Publish
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_cmdvel2gazebo_std_msgs_Float64> Pub_cmdvel2gazebo_9;

// For Block cmdvel2gazebo/Publish1
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_cmdvel2gazebo_std_msgs_Float64> Pub_cmdvel2gazebo_18;

// For Block cmdvel2gazebo/Publish2
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_cmdvel2gazebo_std_msgs_Float64> Pub_cmdvel2gazebo_24;

// For Block cmdvel2gazebo/Publish3
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_cmdvel2gazebo_std_msgs_Float64> Pub_cmdvel2gazebo_25;

void slros_node_init(int argc, char** argv);

#endif
