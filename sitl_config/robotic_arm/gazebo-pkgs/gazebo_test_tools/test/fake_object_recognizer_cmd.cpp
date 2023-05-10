#include <gazebo_test_tools/FakeObjectRecognizer.h>
#include <ros/ros.h>

#define DEFAULT_RECOGNIZE_OBJECT_TOPIC "/gazebo/recognize_object"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_object_recognizer_cmd");

	if (argc != 3)
	{
		ROS_INFO_STREAM("usage: "<<argv[0]<<" <object-name> <republish-flag>");
		return 1;
	}

	ros::NodeHandle n;
	std::string RECOGNIZE_OBJECT_TOPIC;

	n.param<std::string>("gazebo_test_tools/recognize_object_service", RECOGNIZE_OBJECT_TOPIC, DEFAULT_RECOGNIZE_OBJECT_TOPIC);
	ROS_INFO("Got recognize object service topic name: <%s>", RECOGNIZE_OBJECT_TOPIC.c_str());

	ros::ServiceClient client = n.serviceClient<gazebo_test_tools::RecognizeGazeboObject>(RECOGNIZE_OBJECT_TOPIC);
    gazebo_test_tools::RecognizeGazeboObject srv;
	srv.request.name = argv[1];
    srv.request.republish = atoi(argv[2]);

	if (client.call(srv))
	{
		ROS_INFO("Result:");
		std::cout<<srv.response<<std::endl;
	}
	else
	{
		ROS_ERROR("Failed to call service %s",RECOGNIZE_OBJECT_TOPIC.c_str());
		return 1;
	}

	return 0;
}
