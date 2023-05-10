#include <object_msgs_tools/ObjectTFBroadcaster.h>

/***
 * Launches a node which runs the ObjectTFBroadcaster.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
int main(int argc, char**argv){
	ros::init(argc, argv, "object_tf_broadcaster");

	ros::NodeHandle priv("~");
	ros::NodeHandle pub("");

    // use same namespace which is used by launch file to load ROS params
    ros::NodeHandle paramNH("object_tf_broadcaster");	

    std::string REGISTER_OBJECT_SERVICE="/register_object";
	paramNH.param<std::string>("register_object_service", REGISTER_OBJECT_SERVICE, REGISTER_OBJECT_SERVICE);

	std::string OBJECT_TOPIC="/gazebo_world/object";
	paramNH.param<std::string>("object_topic", OBJECT_TOPIC, OBJECT_TOPIC);
	
    std::string OBJECT_SERVICE="/gazebo_world/object_info";
	paramNH.param<std::string>("object_service", OBJECT_SERVICE, OBJECT_SERVICE);
	
	int PUBLISH_TF_RATE=10;
	paramNH.param<int>("publish_tf_rate", PUBLISH_TF_RATE, PUBLISH_TF_RATE);
	
    int QUERY_OBJECT_INFO_RATE=1;
	paramNH.param<int>("query_object_info_rate", QUERY_OBJECT_INFO_RATE, QUERY_OBJECT_INFO_RATE);

    ROS_INFO_STREAM("Starting ObjectTFBroadcaster with topics: register="<<REGISTER_OBJECT_SERVICE 
        <<", object="<<OBJECT_TOPIC<<", object service="<<OBJECT_SERVICE<<" tf rate ="
        <<PUBLISH_TF_RATE<<", query info rate: "<<QUERY_OBJECT_INFO_RATE);

    object_msgs_tools::ObjectTFBroadcaster broadcaster(
        pub,
        REGISTER_OBJECT_SERVICE,
        PUBLISH_TF_RATE,
        QUERY_OBJECT_INFO_RATE,
        OBJECT_TOPIC,
        OBJECT_SERVICE);

    // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    // spinner.spin(); // spin() will not return until the node has been shutdown
	ros::spin();
    ROS_INFO("Exit ObjectTFBroadcater");
    return 0;
}
