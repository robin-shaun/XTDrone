/*
 * Copyright 2012 Open Source Robotics Foundation
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

// *************************************************************
// DEPRECATED
// This class has been renamed to gazebo_ros_joint_pose_trajectory
// *************************************************************

#ifndef GAZEBO_ROS_JOINT_TRAJECTORY_PLUGIN_HH
#define GAZEBO_ROS_JOINT_TRAJECTORY_PLUGIN_HH

#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

#undef ENABLE_SERVICE
#ifdef ENABLE_SERVICE
#include <gazebo_msgs/SetJointTrajectory.h>
#endif

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  class GazeboRosJointPoseTrajectory : public ModelPlugin // replaced with GazeboROSJointPoseTrajectory
  {
    /// \brief Constructor
    public: GazeboRosJointPoseTrajectory();

    /// \brief Destructor
    public: virtual ~GazeboRosJointPoseTrajectory();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void SetTrajectory(
      const trajectory_msgs::JointTrajectory::ConstPtr& trajectory);
#ifdef ENABLE_SERVICE
    private: bool SetTrajectory(
      const gazebo_msgs::SetJointTrajectory::Request& req,
      const gazebo_msgs::SetJointTrajectory::Response& res);
#endif
    private: void UpdateStates();

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    /// \brief pose should be set relative to this link (default to "world")
    private: physics::LinkPtr reference_link_;
    private: std::string reference_link_name_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Subscriber sub_;
    private: ros::ServiceServer srv_;
    private: bool has_trajectory_;

    /// \brief ros message
    private: trajectory_msgs::JointTrajectory trajectory_msg_;
    private: bool set_model_pose_;
    private: geometry_msgs::Pose model_pose_;

    /// \brief topic name
    private: std::string topic_name_;
    private: std::string service_name_;

    /// \brief A mutex to lock access to fields that are
    /// used in message callbacks
    private: boost::mutex update_mutex;

    /// \brief save last_time
    private: common::Time last_time_;

    // trajectory time control
    private: common::Time trajectory_start;
    private: unsigned int trajectory_index;

    // rate control
    private: double update_rate_;
    private: bool disable_physics_updates_;
    private: bool physics_engine_enabled_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    private: std::vector<gazebo::physics::JointPtr> joints_;
    private: std::vector<trajectory_msgs::JointTrajectoryPoint> points_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    private: trajectory_msgs::JointTrajectory joint_trajectory_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
  };
}
#endif
