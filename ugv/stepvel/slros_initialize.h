#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block stepvel/Publish for Leader Vel
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_stepvel_geometry_msgs_Twist> Pub_stepvel_27;

// For Block stepvel/Connstant Velocity
extern SimulinkParameterGetter<real64_T, double> ParamGet_stepvel_93;

// For Block stepvel/Steering angle
extern SimulinkParameterGetter<real64_T, double> ParamGet_stepvel_92;

void slros_node_init(int argc, char** argv);

#endif
