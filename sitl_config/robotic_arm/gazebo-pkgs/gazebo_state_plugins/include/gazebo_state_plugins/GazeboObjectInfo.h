#ifndef GAZEBO_GAZEBOOBJECTINFO_H
#define GAZEBO_GAZEBOOBJECTINFO_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs/ObjectInfoRequest.h>
#include <object_msgs/ObjectInfoResponse.h>

#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{

/**
 * A Gazebo *World* plugin which publishes information about objects in the gazebo world
 * via a ROS message (object_msgs::Object). Until now, only bounding box
 * representations of the objects are supported.
 *
 * Also provides a service to request information about an object directly.
 *
 * The plugin parameters can be specified in a YAML file,
 * which needs to be loaded onto the parameter server
 * under **namespace gazebo_state_plugins**, as follows:
 *
 * ```
 * # Continuously publish info of gazebo objects?
 * publish_world_objects: true
 * 
 * # the topic onto which to publish object info of
 * # type object_msgs/Object
 * world_objects_topic: "/gazebo_objects"
 * 
 * # The topic onto which to publish the service
 * # which can be used to request object info    
 * request_object_service: "/gazebo_objects/get_info"
 * 
 * # frame of the objects poses that will be published.
 * # Should usually be 'world'
 * objects_frame_id: "world"
 * ```
 *
 * \author Jennifer Buehler
 */
class GazeboObjectInfo : public WorldPlugin
{
public: 
	typedef object_msgs::Object ObjectMsg;
	typedef object_msgs::ObjectInfo ObjectInfoMsg;

	GazeboObjectInfo();

	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:

	bool requestObject(object_msgs::ObjectInfo::Request  &req, object_msgs::ObjectInfo::Response &res); 
	
	void advertEvent(const ros::TimerEvent& e); 


	//publishes a model made up of its bounding boxes
	void onWorldUpdate(); 

	shape_msgs::SolidPrimitive * getSolidPrimitive(physics::CollisionPtr& c); 

	//returns a model made up of its bounding boxes
	ObjectMsg createBoundingBoxObject(physics::ModelPtr& model, bool include_shape);
	
private:
	//ros::NodeHandle node;
	bool PUBLISH_OBJECTS;
	std::string WORLD_OBJECTS_TOPIC;
	std::string REQUEST_OBJECTS_TOPIC;
	std::string ROOT_FRAME_ID;
	
	physics::WorldPtr world;

	event::ConnectionPtr update_connection;
	ros::Publisher object_pub;

	ros::ServiceServer request_object_srv;
	
	ros::Timer publishTimer;
			
	std::vector<ObjectMsg> lastGeneratedObjects;
	bool reGenerateObjects;
};

}

#endif  // GAZEBO_GAZEBOOBJECTINFO_H
