#include <ros/ros.h>
#include <object_msgs/RegisterObject.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "register_object_client");
    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");
   
    if (!pub.hasParam("/object_tf_broadcaster/register_object_service"))
    {
        ROS_ERROR("Missing parameter register_object_service");
        return 0;
    }
    std::string REGISTER_OBJECT_SERVICE; 
	priv.param<std::string>("/object_tf_broadcaster/register_object_service", REGISTER_OBJECT_SERVICE, REGISTER_OBJECT_SERVICE);
	ROS_INFO("Using register object service name: <%s>", REGISTER_OBJECT_SERVICE.c_str());

	ros::ServiceClient client = pub.serviceClient<object_msgs::RegisterObject>(REGISTER_OBJECT_SERVICE);
    object_msgs::RegisterObject srv;
	srv.request.name = argv[1];
	if (client.call(srv))
	{
		ROS_INFO("Result:");
		std::cout<<srv.response<<std::endl;
	}
	else
	{
		ROS_ERROR("Failed to call service %s",REGISTER_OBJECT_SERVICE.c_str());
	}
    return 0;
}
