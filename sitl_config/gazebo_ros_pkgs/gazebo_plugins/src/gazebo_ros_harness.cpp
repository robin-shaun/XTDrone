/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

#include "gazebo_plugins/gazebo_ros_harness.h"

namespace gazebo
{

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosHarness)

/////////////////////////////////////////////////
GazeboRosHarness::GazeboRosHarness()
{
}

/////////////////////////////////////////////////
GazeboRosHarness::~GazeboRosHarness()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();

  this->rosnode_->shutdown();
  delete this->rosnode_;
}

/////////////////////////////////////////////////
// Load the controller
void GazeboRosHarness::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Load the plugin
  HarnessPlugin::Load(_parent, _sdf);

  this->robotNamespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace_ = _sdf->Get<std::string>("robotNamespace") + "/";

  // Init ROS
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("harness", "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + _parent->GetName() + "/harness/velocity", 1,
    boost::bind(&GazeboRosHarness::OnVelocity, this, _1),
    ros::VoidPtr(), &this->queue_);
  this->velocitySub_ = this->rosnode_->subscribe(so);

  so = ros::SubscribeOptions::create<std_msgs::Bool>(
    "/" + _parent->GetName() + "/harness/detach", 1,
    boost::bind(&GazeboRosHarness::OnDetach, this, _1),
    ros::VoidPtr(), &this->queue_);
  this->detachSub_ = this->rosnode_->subscribe(so);

  // Custom Callback Queue
  this->callbackQueueThread_ =
    boost::thread(boost::bind(&GazeboRosHarness::QueueThread, this));
}

/////////////////////////////////////////////////
void GazeboRosHarness::OnVelocity(const std_msgs::Float32::ConstPtr &msg)
{
  // Set the target winch velocity
  this->SetWinchVelocity(msg->data);
}

/////////////////////////////////////////////////
void GazeboRosHarness::OnDetach(const std_msgs::Bool::ConstPtr &msg)
{
  // Detach if true
  if (msg->data)
    this->Detach();
}

/////////////////////////////////////////////////
void GazeboRosHarness::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
