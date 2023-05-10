#ifndef GAZEBO_GAZEBOPLUGINLOADER_H
#define GAZEBO_GAZEBOPLUGINLOADER_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

/**
 * System plugin which can be used to load World plugins
 * dynamically, without the need of having to put them in
 * the Gazebo world file.
 *
 * The plugins to be loaded can be specified in a YAML file,
 * which needs to be loaded onto the parameter server
 * under **namespace gazebo_state_plugins**, as follows:
 *
 * ```
 *  world_name: "default"
 *  world_plugins:
 *      - name: gazebo_object_info
 *        file: libgazebo_object_info.so
 *      - name: gazebo_map_publisher
 *        file: libgazebo_map_publisher.so
 * ```
 * 
 * **Usage:**
 *
 * This plugin is a system plugin which has to be loaded
 * together with gazebo, e.g.:
 *
 * ``gzserver -s libgazebo_world_plugin_loader.so``
 *
 * **Limitation:**
 *
 * The method
 * *WorldPlugin::Load(physics::WorldPtr, sdf::ElementPtr)*
 * on the plugins will be called with a NULL sdf::ElementPtr.
 * So the plugins should alternatively support reading their
 * parameters from the ROS parameter server.
 *
 * \author Jennifer Buehler
 */
class GazeboPluginLoader : public SystemPlugin
{
public: 
	GazeboPluginLoader();

	void Load(int argc, char ** argv);

private:
	void onWorldCreate(); 
	event::ConnectionPtr update_connection;
};

}
#endif  // GAZEBO_GAZEBOPLUGINLOADER_H
