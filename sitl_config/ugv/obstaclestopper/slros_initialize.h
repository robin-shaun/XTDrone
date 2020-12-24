#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block obstacleStopper/Subscribe
extern SimulinkSubscriber<std_msgs::Float64, SL_Bus_obstacleStopper_std_msgs_Float64> Sub_obstacleStopper_12;

// For Block obstacleStopper/Subscribe1
extern SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Sub_obstacleStopper_13;

// For Block obstacleStopper/Subscribe2
extern SimulinkSubscriber<geometry_msgs::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Sub_obstacleStopper_39;

// For Block obstacleStopper/Publish
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_obstacleStopper_geometry_msgs_Twist> Pub_obstacleStopper_17;

void slros_node_init(int argc, char** argv);

#endif
