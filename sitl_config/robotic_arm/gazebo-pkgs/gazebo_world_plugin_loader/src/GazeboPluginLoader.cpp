#include <gazebo_world_plugin_loader/GazeboPluginLoader.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsIface.hh>

using gazebo::GazeboPluginLoader;

GZ_REGISTER_SYSTEM_PLUGIN(GazeboPluginLoader)

GazeboPluginLoader::GazeboPluginLoader() : 
    SystemPlugin()
{
    update_connection = event::Events::ConnectWorldCreated(boost::bind(&GazeboPluginLoader::onWorldCreate, this));
}

void GazeboPluginLoader::Load(int argc, char ** argv){
}

void GazeboPluginLoader::onWorldCreate(){
    ROS_INFO("On-World-Create: Reading list of Gazebo world plugins to be loaded from parameters...");

    ros::NodeHandle node("/gazebo_state_plugins");

    ROS_INFO_STREAM("Reading parameters from namespace "<<node.getNamespace());

    std::string world_name;
    node.getParam("world_name", world_name);
    ROS_INFO_STREAM("Loading plugins for world '"<<world_name<<"'");

    physics::WorldPtr world = gazebo::physics::get_world(world_name);
    if (!world)
    {
        ROS_ERROR("Could not obtain pointer to world from system plugin, so won't be able to load plugins");
        return;
    }

    XmlRpc::XmlRpcValue world_plugins;
    node.getParam("world_plugins", world_plugins);
    if (world_plugins.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Parameter world_plugins should be specified as an array. Gazebo world plugins won't be loaded.");
      return;
    }

    /* get the list of names */
    for (int i = 0 ; i < world_plugins.size() ; ++i)
    {
      if (!world_plugins[i].hasMember("name") || !world_plugins[i].hasMember("file"))
      {
            ROS_ERROR("World plugin parameter specification should have 'name' and 'file'. Gazebo world plugins won't be loaded.");
            continue;
      }
      std::string name=world_plugins[i]["name"];
      std::string file=world_plugins[i]["file"];
      sdf::ElementPtr _sdf;  // will be initialized to NULL
      world->LoadPlugin(file, name, _sdf);
    }

    ROS_INFO("Eligible plugins loaded.");
}
