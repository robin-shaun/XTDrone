/*
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
/**
 *  \author David Leins
 *  \date 28th of November 2018
 *  \desc Gazebo ros plugin to steer an actor with twist messages over ros. Based on actor_plugin. 
 */

#ifndef GAZEBO_ROS_ACTOR_POSE_PLUGIN_HH
#define GAZEBO_ROS_ACTOR_POSE_PLUGIN_HH

// Ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros_actor_cmd_pose_plugin_msgs/ToggleActorWaving.h>

// Ignition
#include <ignition/math.hh>

// Gazebo
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/util/system.hh"

#include <gazebo_plugins/gazebo_ros_utils.h>

// STD
#include <string>
#include <vector>
#include <functional>

namespace gazebo
{

   class GAZEBO_VISIBLE ActorPluginRos : public ModelPlugin
   {
      /// \brief Constructor
      public: ActorPluginRos();

      /// \brief Load the controller
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      // Documentation Inherited.
      public: virtual void Reset();

      /// \brief Function that is called every update cycle.
      /// \param[in] _info Timing information
      private: void OnUpdate(const common::UpdateInfo &_info);

      // /// \brief Callback for newly received vel command
      // private: void NewVelCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      /// \brief Callback for newly received pose command
      private: void CmdPoseCallback(const geometry_msgs::Point::ConstPtr& cmd_msg);

      /// \brief Choose New Target
      private: void ChooseNewTarget();

      /// \brief Service to toggle the waving animation
      private: bool ToggleWaveAnimation(ros_actor_cmd_pose_plugin_msgs::ToggleActorWaving::Request &req, ros_actor_cmd_pose_plugin_msgs::ToggleActorWaving::Response &res);

      /// \brief Pointer to the parent actor.
      private: physics::ActorPtr actor;

      /// \brief Pointer to the world, for convenience.
      private: physics::WorldPtr world;
      /// \brief flag to detect the topic.
      private: bool GET_CMD_FLAG = false;
      /// \brief Pointer to the sdf element.
      private: sdf::ElementPtr sdf;

      /// \brief Velocity of the actor
      private: ignition::math::Vector3d velocity;

      /// \brief List of connections
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Current target location
      private: ignition::math::Vector3d target;

      /// \brief initial postion
      private: ignition::math::Pose3d init_pose;

      /// \brief Last received linear vel command.
      private: ignition::math::Vector3d last_linear;

      /// \brief Last received angle command.
      private: ignition::math::Angle last_angle;

      /// \brief Time scaling factor. Used to coordinate translational motion
      /// with the actor's walking animation.
      private: double animationFactor = 1.0;

      /// \brief Time of the last update.
      private: common::Time lastUpdate;

      /// \brief Custom trajectory info.
      private: physics::TrajectoryInfoPtr trajectoryInfo;

      private: GazeboRosPtr gazebo_ros_;
      // private: ros::Subscriber cmd_vel_subscriber_;
      private: ros::Subscriber cmd_pose_subscriber_;
      private: ros::ServiceServer wave_toggle_service_;

      private: bool wave_toggled;

   };

}

#endif
