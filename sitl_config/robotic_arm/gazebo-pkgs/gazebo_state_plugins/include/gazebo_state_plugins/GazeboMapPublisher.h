#ifndef GAZEBO_GAZEBOMAPPUBLISHER_H
#define GAZEBO_GAZEBOMAPPUBLISHER_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetMapRequest.h>
#include <nav_msgs/GetMapResponse.h>
#include <nav_msgs/OccupancyGrid.h>

namespace gazebo
{

/**
 * Publishes a map of the grid in gazebo to a topic that the map is supposed to be published to.
 *
 * A service is also provided to request a map (service name can be specified with ros paramter
 * "request_map_service") of type nav_msgs/GetMap.
 *
 * The plugin parameters can be specified in a YAML file,
 * which needs to be loaded onto the parameter server
 * under **namespace gazebo_state_plugins**, as follows:
 *
 * ```
 * # the topic onto which to continuously publish
 * # the map.
 * publish_map_topic: "/gazebo/map"
 * 
 * # frequency to publish the map
 * map_pub_frequency: 1
 * 
 * # topic onto which to publish the metadata
 * # information of the map. Only will be published
 * # if publish_map_topic is not empty.
 * map_metadata_topic: "/map_metadata"
 * 
 * # topic onto which to provide a service to
 * # request the map.
 * request_map_service: "/dynamic_map"
 * 
 * # The frame ID to assign to the map    
 * map_frame_id: "/map"
 * 
 * # the name of the robot in Gazebo
 * robot_name: "robot"
 * ```
 *
 * \author Jennifer Buehler
 */
class GazeboMapPublisher : public WorldPlugin
{
public: 

	GazeboMapPublisher();
  	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:

	bool requestMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res); 

	void advertEvent(const ros::TimerEvent& e); 

	void onWorldUpdate(); 

    // forward declaration
    class CollisionMapRequest;

	//see http://gazebosim.org/wiki/Tutorials/1.4/custom_messages#Collision_Map_Creator_Plugin
	bool createMap(const CollisionMapRequest &msg, const std::string& map_frame, nav_msgs::OccupancyGrid& map);

	CollisionMapRequest getCollisionRequest();

	nav_msgs::MapMetaData getMetaData(); 

	nav_msgs::OccupancyGrid getMap(); 
	
    //map resolution (m/cell)
	float map_resolution;

    //height of obstacles to consider for generating the map
	float map_height;
	double map_offset_x;
	double map_offset_y;
	double map_len_x;
	double map_len_y;

    ros::NodeHandle node;

	std::string MAP_TOPIC;
	std::string MAP_FRAME;
	std::string METADATA_TOPIC;
	std::string REQUEST_MAP_SERVICE;
	std::string ROBOT_NAME;
	float MAP_PUB_FREQ;
	
	physics::WorldPtr world;

	event::ConnectionPtr update_connection;
	ros::Publisher map_pub;
	ros::Publisher meta_pub;

	ros::ServiceServer request_map_srv;
		
	ros::Timer publishTimer;

	bool worldChangedSinceLastAdvert; 
};

}
#endif  // GAZEBO_GAZEBOMAPPUBLISHER_H
