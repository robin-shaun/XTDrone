#ifndef GAZEBO_ROS_WIND_PLUGIN_XTDRONE_H
#define GAZEBO_ROS_WIND_PLUGIN_XTDRONE_H

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <sdf/Param.hh>

#include "Wind.pb.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>




namespace gazebo {

/// \brief This gazebo plugin simulates wind acting on a model based on ROS message
class GazeboROSWindPluginXTDrone : public WorldPlugin {
 public:
  GazeboROSWindPluginXTDrone()
   : WorldPlugin(),
   namespace_(""),
   wind_pub_topic_("world_wind"),
   wind_ros_sub_topic_("wind_xtdrone"),
   frame_id_("world"),
   pub_interval_(0.5),
   gazebo_node_(NULL) {}

  virtual ~GazeboROSWindPluginXTDrone();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);


private:
 /// \brief Pointer to the update event connection.
 event::ConnectionPtr update_connection_;

 physics::WorldPtr world_;

 std::string namespace_;

 std::string frame_id_;
 std::string wind_pub_topic_;
 std::string wind_ros_sub_topic_;

 double pub_interval_;

 // wind stuff
 void windCallback(const geometry_msgs::Twist::ConstPtr& wind_msg);
 double wind_x;
 double wind_y;
 double wind_z;
 bool alive_;

 boost::mutex lock;

 common::Time last_time_;

 transport::NodePtr gazebo_node_;
 transport::PublisherPtr wind_pub_;
 physics_msgs::msgs::Wind wind_msg;

//  ros::NodeHandle *rosnode_;
boost::shared_ptr<ros::NodeHandle> rosnode_;
 ros::Subscriber wind_sub_;

 /// \brief Custom Callback Queue
 ros::CallbackQueue queue_;
 boost::thread callback_queue_thread_;

/// \brief Custom Callback Queue thread
 boost::thread callbackQueueThread_;

 /// \brief Queu to handle callbacks.
 void QueueThread();
};
}




#endif
