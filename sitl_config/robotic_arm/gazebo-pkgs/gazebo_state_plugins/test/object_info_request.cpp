#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs/ObjectInfoRequest.h>
#include <object_msgs/ObjectInfoResponse.h>
#include <object_msgs/Object.h>
#include <iostream>

#define DEFAULT_REQUEST_OBJECTS_TOPIC "world/request_object"

int main (int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_object_info_client");

	if (argc != 2)
	{
		ROS_INFO_STREAM("usage: "<<argv[0]<<" <object-name>");
		return 1;
	}

	ros::NodeHandle n;
	std::string REQUEST_OBJECTS_TOPIC;

	n.param<std::string>("gazebo_state_plugins/request_object_service", REQUEST_OBJECTS_TOPIC, DEFAULT_REQUEST_OBJECTS_TOPIC);
	ROS_INFO("Got objects topic name: <%s>", REQUEST_OBJECTS_TOPIC.c_str());

	ros::ServiceClient client = n.serviceClient<object_msgs::ObjectInfo>(REQUEST_OBJECTS_TOPIC);
	object_msgs::ObjectInfo srv;
	srv.request.name = argv[1];
    srv.request.get_geometry=true;

	if (client.call(srv) && srv.response.success)
	{
		ROS_INFO("Result:");
		std::cout<<srv.response<<std::endl;
	}
	else
	{
		ROS_ERROR("Failed to call service %s, success flag: %i",REQUEST_OBJECTS_TOPIC.c_str(),srv.response.success);
		return 1;
	}

	return 0;
}
