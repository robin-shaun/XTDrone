/*
 * Change the wind through ROS message
 * Andy Zhuo, XTDrone
 * 2023.06.30
*/

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include "gazebo_ros_wind_plugin_xtdrone.h"
#include "gazebo/common/common.hh"

namespace gazebo {

GazeboROSWindPluginXTDrone::~GazeboROSWindPluginXTDrone() {
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callbackQueueThread_.join();

  update_connection_->~Connection();
}

void GazeboROSWindPluginXTDrone::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) {
  world_ = world;

  if (!sdf->HasElement("robotNamespace")) {
    ROS_INFO_NAMED("wind_xtdrone", "wind plugin missing <robotNamespace>, defaults to %s", namespace_.c_str());
  }
  else{
    namespace_ = sdf->Get<std::string>("robotNamespace");
  }

  if (!sdf->HasElement("frameId")){
    ROS_INFO_NAMED("wind_xtdrone", "wind plugin missing <frameId>, defaults to %s", frame_id_.c_str());
  }
  else{
    frame_id_ = sdf->Get<std::string>("frameId");
  }

  if (!sdf->HasElement("windPubTopic")){
    ROS_INFO_NAMED("wind_xtdrone", "wind plugin missing <windPubTopic>, defaults to %s", wind_pub_topic_.c_str());
  }
  else{
    wind_pub_topic_ = sdf->Get<std::string>("windPubTopic");
  }

  double pub_rate = 2.0;
  if (!sdf->HasElement("publishRate")){
    ROS_INFO_NAMED("wind_xtdrone", "wind plugin missing <publishRate>, defaults to %f", pub_rate);
  }
  else{
    pub_rate = sdf->Get<double>("publishRate");
  }
  pub_interval_ = (pub_rate > 0.0) ? 1/pub_rate : 0.0;

  wind_x = 0;
  wind_y = 0;
  wind_z = 0;


  /* Gazebo Node */
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init(namespace_);

  wind_pub_ = gazebo_node_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("imu", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  /* ROS Node */
  rosnode_.reset(new ros::NodeHandle(namespace_));
  // ROS: Subscribe to the wind command topic (usually "wind_xtdrone")
  ROS_INFO_NAMED("wind_xtdrone", "XTdrone wind plugin try to subscribe to ROS topic: %s", wind_ros_sub_topic_.c_str());
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(wind_ros_sub_topic_, 1,
              boost::bind(&GazeboROSWindPluginXTDrone::windCallback, this, _1),
              ros::VoidPtr(), &queue_);
  wind_sub_ = rosnode_->subscribe(so);

  // start custom queue for diff drive
  this->callback_queue_thread_ =
      boost::thread ( boost::bind ( &GazeboROSWindPluginXTDrone::QueueThread, this ) );

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboROSWindPluginXTDrone::OnUpdate, this, _1));

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
#endif
}

// This gets called by the world update start event.
void GazeboROSWindPluginXTDrone::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif
  if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
    return;
  }
  last_time_ = now;

  /* processing */
  gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
  wind_v->set_x(wind_x);
  wind_v->set_y(wind_y);
  wind_v->set_z(wind_z);

  // ROS_WARN("wind-pub: [%f, %f, %f]", wind_x, wind_y, wind_z);

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now.Double() * 1e6);
  wind_msg.set_allocated_velocity(wind_v);

  wind_pub_->Publish(wind_msg);
}

void GazeboROSWindPluginXTDrone::windCallback(const geometry_msgs::Twist::ConstPtr& wind_msg)
{
  // boost::mutex::scoped_lock scoped_lock ( lock );
  wind_x = wind_msg->linear.x;
  wind_y = wind_msg->linear.y;
  wind_z = wind_msg->linear.z;

  // ROS_WARN("wind-sub: [%f, %f, %f]", wind_x, wind_y, wind_z);
}

void GazeboROSWindPluginXTDrone::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
    this->queue_.callAvailable(ros::WallDuration(timeout));
}


GZ_REGISTER_WORLD_PLUGIN(GazeboROSWindPluginXTDrone);
}
