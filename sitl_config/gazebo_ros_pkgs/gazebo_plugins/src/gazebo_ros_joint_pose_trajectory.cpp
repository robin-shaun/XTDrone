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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#include <string>
#include <stdlib.h>
#include <tf/tf.h>

#include <gazebo_plugins/gazebo_ros_joint_pose_trajectory.h>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointPoseTrajectory);

////////////////////////////////////////////////////////////////////////////////
// Constructor
ROS_DEPRECATED GazeboRosJointPoseTrajectory::GazeboRosJointPoseTrajectory()  // replaced with GazeboROSJointPoseTrajectory
{

  this->has_trajectory_ = false;
  this->trajectory_index = 0;
  this->joint_trajectory_.points.clear();
  this->physics_engine_enabled_ = true;
  this->disable_physics_updates_ = true;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosJointPoseTrajectory::~GazeboRosJointPoseTrajectory()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosJointPoseTrajectory::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  // save pointers
  this->model_ = _model;
  this->sdf = _sdf;
  this->world_ = this->model_->GetWorld();

  // this->world_->SetGravity(ignition::math::Vector3d(0, 0, 0));

  // load parameters
  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("serviceName"))
  {
    // default
    this->service_name_ = "set_joint_trajectory";
  }
  else
    this->service_name_ = this->sdf->Get<std::string>("serviceName");

  if (!this->sdf->HasElement("topicName"))
  {
    // default
    this->topic_name_ = "set_joint_trajectory";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_INFO_NAMED("joint_pose_trajectory", "joint trajectory plugin missing <updateRate>, defaults"
             " to 0.0 (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->Get<double>("updateRate");

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("joint_pose_trajectory", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosJointPoseTrajectory::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosJointPoseTrajectory::LoadThread()
{
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);

  if (this->topic_name_ != "")
  {
    ros::SubscribeOptions trajectory_so =
      ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>(
      this->topic_name_, 100, boost::bind(
      &GazeboRosJointPoseTrajectory::SetTrajectory, this, _1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(trajectory_so);
  }

#ifdef ENABLE_SERVICE
  if (this->service_name_ != "")
  {
    ros::AdvertiseServiceOptions srv_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetJointTrajectory>(
      this->service_name_,
      boost::bind(&GazeboRosJointPoseTrajectory::SetTrajectory, this, _1, _2),
      ros::VoidPtr(), &this->queue_);
    this->srv_ = this->rosnode_->advertiseService(srv_aso);
  }
#endif

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  // start custom queue for joint trajectory plugin ros topics
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboRosJointPoseTrajectory::QueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosJointPoseTrajectory::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
// set joint trajectory
void GazeboRosJointPoseTrajectory::SetTrajectory(
  const trajectory_msgs::JointTrajectory::ConstPtr& trajectory)
{
  boost::mutex::scoped_lock lock(this->update_mutex);

  this->reference_link_name_ = trajectory->header.frame_id;
  // do this every time a new joint trajectory is supplied,
  // use header.frame_id as the reference_link_name_
  if (this->reference_link_name_ != "world" &&
      this->reference_link_name_ != "/map" &&
      this->reference_link_name_ != "map")
  {
    physics::EntityPtr ent =
#if GAZEBO_MAJOR_VERSION >= 8
      this->world_->EntityByName(this->reference_link_name_);
#else
      this->world_->GetEntity(this->reference_link_name_);
#endif
    if (ent)
      this->reference_link_ = boost::dynamic_pointer_cast<physics::Link>(ent);
    if (!this->reference_link_)
    {
      ROS_ERROR_NAMED("joint_pose_trajectory", "ros_joint_trajectory plugin needs a reference link [%s] as"
                " frame_id, aborting.\n", this->reference_link_name_.c_str());
      return;
    }
    else
    {
      this->model_ = this->reference_link_->GetParentModel();
      ROS_DEBUG_NAMED("joint_pose_trajectory", "test: update model pose by keeping link [%s] stationary"
                " inertially", this->reference_link_->GetName().c_str());
    }
  }

  // copy joint configuration into a map
  unsigned int chain_size = trajectory->joint_names.size();
  this->joints_.resize(chain_size);
  for (unsigned int i = 0; i < chain_size; ++i)
  {
    this->joints_[i] = this->model_->GetJoint(trajectory->joint_names[i]);
  }

  unsigned int points_size = trajectory->points.size();
  this->points_.resize(points_size);
  for (unsigned int i = 0; i < points_size; ++i)
  {
    this->points_[i].positions.resize(chain_size);
    this->points_[i].time_from_start = trajectory->points[i].time_from_start;
    for (unsigned int j = 0; j < chain_size; ++j)
    {
      this->points_[i].positions[j] = trajectory->points[i].positions[j];
    }
  }

  // trajectory start time
  this->trajectory_start = gazebo::common::Time(trajectory->header.stamp.sec,
                                                trajectory->header.stamp.nsec);
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif
  if (this->trajectory_start < cur_time)
    this->trajectory_start = cur_time;

  // update the joint trajectory to play
  this->has_trajectory_ = true;
  // reset trajectory_index to beginning of new trajectory
  this->trajectory_index = 0;

  if (this->disable_physics_updates_)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->physics_engine_enabled_ = this->world_->PhysicsEnabled();
    this->world_->SetPhysicsEnabled(false);
#else
    this->physics_engine_enabled_ = this->world_->GetEnablePhysicsEngine();
    this->world_->EnablePhysicsEngine(false);
#endif
  }
}

#ifdef ENABLE_SERVICE
bool GazeboRosJointPoseTrajectory::SetTrajectory(
  const gazebo_msgs::SetJointTrajectory::Request& req,
  const gazebo_msgs::SetJointTrajectory::Response& res)
{
  boost::mutex::scoped_lock lock(this->update_mutex);

  this->model_pose_ = req.model_pose;
  this->set_model_pose_ = req.set_model_pose;

  this->reference_link_name_ = req.joint_trajectory.header.frame_id;
  // do this every time a new joint_trajectory is supplied,
  // use header.frame_id as the reference_link_name_
  if (this->reference_link_name_ != "world" &&
      this->reference_link_name_ != "/map" &&
      this->reference_link_name_ != "map")
  {
    physics::EntityPtr ent =
#if GAZEBO_MAJOR_VERSION >= 8
      this->world_->EntityByName(this->reference_link_name_);
#else
      this->world_->GetEntity(this->reference_link_name_);
#endif
    if (ent)
      this->reference_link_ = boost::dynamic_pointer_cast<physics::Link>(ent);
    if (!this->reference_link_)
    {
      ROS_ERROR_NAMED("joint_pose_trajectory", "ros_joint_trajectory plugin specified a reference link [%s]"
                " that does not exist, aborting.\n",
                this->reference_link_name_.c_str());
      ROS_DEBUG_NAMED("joint_pose_trajectory", "will set model [%s] configuration, keeping model root link"
                " stationary.", this->model_->GetName().c_str());
      return false;
    }
    else
      ROS_DEBUG_NAMED("joint_pose_trajectory", "test: update model pose by keeping link [%s] stationary"
                " inertially", this->reference_link_->GetName().c_str());
  }

#if GAZEBO_MAJOR_VERSION >= 8
  this->model_ =  this->world_->ModelByName(req.model_name);
#else
  this->model_ =  this->world_->GetModel(req.model_name);
#endif
  if (!this->model_)  // look for it by frame_id name
  {
    this->model_ = this->reference_link_->GetParentModel();
    if (this->model_)
    {
      ROS_INFO_NAMED("joint_pose_trajectory", "found model[%s] by link name specified in frame_id[%s]",
        this->model_->GetName().c_str(),
        req.joint_trajectory.header.frame_id.c_str());
    }
    else
    {
      ROS_WARN_NAMED("joint_pose_trajectory", "no model found by link name specified in frame_id[%s],"
               "  aborting.", req.joint_trajectory.header.frame_id.c_str());
      return false;
    }
  }

  // copy joint configuration into a map
  this->joint_trajectory_ = req.joint_trajectory;

  // trajectory start time
  this->trajectory_start = gazebo::common::Time(
    req.joint_trajectory.header.stamp.sec,
    req.joint_trajectory.header.stamp.nsec);

  // update the joint_trajectory to play
  this->has_trajectory_ = true;
  // reset trajectory_index to beginning of new trajectory
  this->trajectory_index = 0;
  this->disable_physics_updates_ = req.disable_physics_updates;
  if (this->disable_physics_updates_)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->physics_engine_enabled_ = this->world_->PhysicsEnabled();
    this->world_->SetPhysicsEnabled(false);
#else
    this->physics_engine_enabled_ = this->world_->GetEnablePhysicsEngine();
    this->world_->EnablePhysicsEngine(false);
#endif
  }

  return true;
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void GazeboRosJointPoseTrajectory::UpdateStates()
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosJointPoseTrajectory::UpdateStates");

  IGN_PROFILE_BEGIN("update");
#endif
  boost::mutex::scoped_lock lock(this->update_mutex);
  if (this->has_trajectory_)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time cur_time = this->world_->SimTime();
#else
    common::Time cur_time = this->world_->GetSimTime();
#endif
    // roll out trajectory via set model configuration
    // gzerr << "i[" << trajectory_index  << "] time "
    //       << trajectory_start << " now: " << cur_time << " : "<< "\n";
    if (cur_time >= this->trajectory_start)
    {
      // @todo:  consider a while loop until the trajectory
      // catches up to the current time
      // gzerr << trajectory_index << " : "  << this->points_.size() << "\n";
      if (this->trajectory_index < this->points_.size())
      {
        ROS_INFO_NAMED("joint_pose_trajectory", "time [%f] updating configuration [%d/%lu]",
          cur_time.Double(), this->trajectory_index, this->points_.size());

        // get reference link pose before updates
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d reference_pose = this->model_->WorldPose();
#else
        ignition::math::Pose3d reference_pose = this->model_->GetWorldPose().Ign();
#endif
        if (this->reference_link_)
        {
#if GAZEBO_MAJOR_VERSION >= 8
          reference_pose = this->reference_link_->WorldPose();
#else
          reference_pose = this->reference_link_->GetWorldPose().Ign();
#endif
        }

        // trajectory roll-out based on time:
        //  set model configuration from trajectory message
        unsigned int chain_size = this->joints_.size();
        if (chain_size ==
          this->points_[this->trajectory_index].positions.size())
        {
          for (unsigned int i = 0; i < chain_size; ++i)
          {
            // this is not the most efficient way to set things
            if (this->joints_[i])
            {
#if GAZEBO_MAJOR_VERSION >= 9
              this->joints_[i]->SetPosition(0,
                this->points_[this->trajectory_index].positions[i], true);
#else
              this->joints_[i]->SetPosition(0,
                this->points_[this->trajectory_index].positions[i]);
#endif
            }
          }

          // set model pose
          if (this->reference_link_)
            this->model_->SetLinkWorldPose(reference_pose,
              this->reference_link_);
          else
            this->model_->SetWorldPose(reference_pose);
        }
        else
        {
          ROS_ERROR_NAMED("joint_pose_trajectory", "point[%u] in JointTrajectory has different number of"
                    " joint names[%u] and positions[%lu].",
                    this->trajectory_index, chain_size,
                    this->points_[this->trajectory_index].positions.size());
        }

        // this->world_->SetPaused(is_paused);  // resume original pause-state
        gazebo::common::Time duration(
          this->points_[this->trajectory_index].time_from_start.sec,
          this->points_[this->trajectory_index].time_from_start.nsec);

        // reset start time for next trajectory point
        this->trajectory_start += duration;
        this->trajectory_index++;  // increment to next trajectory point

        // save last update time stamp
        this->last_time_ = cur_time;
      }
      else  // no more trajectory points
      {
        // trajectory finished
        this->reference_link_.reset();
        this->has_trajectory_ = false;
        if (this->disable_physics_updates_)
        {
#if GAZEBO_MAJOR_VERSION >= 8
          this->world_->SetPhysicsEnabled(this->physics_engine_enabled_);
#else
          this->world_->EnablePhysicsEngine(this->physics_engine_enabled_);
#endif
        }
      }
    }
  }
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosJointPoseTrajectory::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
