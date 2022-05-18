/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: Force Feed Back Ground Truth
   Author: Sachin Chitta and John Hsu
   Date: 1 June 2008
 */

#include <gazebo_plugins/gazebo_ros_f3d.h>
#include <tf/tf.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosF3D);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosF3D::GazeboRosF3D()
{
  this->f3d_connect_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosF3D::~GazeboRosF3D()
{
  this->update_connection_.reset();
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosF3D::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Get the world name.
  this->world_ = _parent->GetWorld();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("f3d", "f3d plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("f3d", "gazebo_ros_f3d plugin error: bodyName: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("f3d", "f3d plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("f3d", "f3d plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
  {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
    // todo: frameName not used
    ROS_INFO_NAMED("f3d", "f3d plugin specifies <frameName> [%s], not used, default to world",this->frame_name_.c_str());
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("f3d", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>(
    this->topic_name_,1,
    boost::bind( &GazeboRosF3D::F3DConnect,this),
    boost::bind( &GazeboRosF3D::F3DDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosF3D::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosF3D::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosF3D::F3DConnect()
{
  this->f3d_connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosF3D::F3DDisconnect()
{
  this->f3d_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosF3D::UpdateChild()
{
  if (this->f3d_connect_count_ == 0)
    return;

  ignition::math::Vector3d torque;
  ignition::math::Vector3d force;

  // get force and torque on body
#if GAZEBO_MAJOR_VERSION >= 8
  force = this->link_->WorldForce();
  torque = this->link_->WorldTorque();
#else
  force = this->link_->GetWorldForce().Ign();
  torque = this->link_->GetWorldTorque().Ign();
#endif

  this->lock_.lock();
  // copy data into wrench message
  this->wrench_msg_.header.frame_id = this->frame_name_;
#if GAZEBO_MAJOR_VERSION >= 8
  this->wrench_msg_.header.stamp.sec = (this->world_->SimTime()).sec;
  this->wrench_msg_.header.stamp.nsec = (this->world_->SimTime()).nsec;
#else
  this->wrench_msg_.header.stamp.sec = (this->world_->GetSimTime()).sec;
  this->wrench_msg_.header.stamp.nsec = (this->world_->GetSimTime()).nsec;
#endif

  this->wrench_msg_.wrench.force.x    = force.X();
  this->wrench_msg_.wrench.force.y    = force.Y();
  this->wrench_msg_.wrench.force.z    = force.Z();
  this->wrench_msg_.wrench.torque.x   = torque.X();
  this->wrench_msg_.wrench.torque.y   = torque.Y();
  this->wrench_msg_.wrench.torque.z   = torque.Z();

  this->pub_.publish(this->wrench_msg_);
  this->lock_.unlock();

}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosF3D::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
