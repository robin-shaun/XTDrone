#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "stepvel";

// For Block stepvel/Publish for Leader Vel
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_stepvel_geometry_msgs_Twist> Pub_stepvel_27;

// For Block stepvel/Connstant Velocity
SimulinkParameterGetter<real64_T, double> ParamGet_stepvel_93;

// For Block stepvel/Steering angle
SimulinkParameterGetter<real64_T, double> ParamGet_stepvel_92;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

