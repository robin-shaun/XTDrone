#include <gazebo/transport/transport.hh>

#include <gazebo_grasp_plugin/msgs/grasp_event.pb.h>
#include <gazebo_grasp_plugin_ros/GazeboGraspEvent.h>
#include <ros/ros.h>

using GraspEventPtr = boost::shared_ptr<const gazebo::msgs::GraspEvent>;

ros::Publisher gGraspEventPublisher;

/**
 * Callback to receive gazebo grasp event messages
 */
void ReceiveGraspMsg(const GraspEventPtr& gzMsg)
{
  ROS_INFO_STREAM("Re-publishing grasp event: " << gzMsg->DebugString());
  gazebo_grasp_plugin_ros::GazeboGraspEvent rosMsg;
  rosMsg.arm      = gzMsg->arm();
  rosMsg.object   = gzMsg->object();
  rosMsg.attached = gzMsg->attached();
  gGraspEventPublisher.publish(rosMsg);
}

int main(int argc, char** argv)
{
  // Initialize ROS and Gazebo transport
  ros::init(argc, argv, "gazebo_grasp_plugin_event_republisher");
  if (!gazebo::transport::init())
  {
    ROS_ERROR("Unable to initialize gazebo transport - is gazebo running?");
    return 1;
  }
  gazebo::transport::run();

  // Subscribe to Gazebo grasp event message
  gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
  gzNode->Init();
  gazebo::transport::SubscriberPtr subscriber;
  try
  {
    subscriber = gzNode->Subscribe("~/grasp_events", &ReceiveGraspMsg);
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM("Error subscribing to topic: " << e.what());
    return 1;
  }

  // Initialize ROS publisher
  ros::NodeHandle   rosNode("~");
  const std::string pubTopic = "grasp_events";
  gGraspEventPublisher =
    rosNode.advertise<gazebo_grasp_plugin_ros::GazeboGraspEvent>(pubTopic, 1);

  ros::spin();
  gazebo::transport::fini();
  return 0;
}
