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

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN5_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN5_HH_

#include <string>
#include <vector>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/util/system.hh"
#include "gazebo/msgs/msgs.hh"
#include <gazebo/gazebo.hh>

#include "Actor.pb.h"
#include "CmdActor.pb.h"

namespace gazebo
{
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

typedef const boost::shared_ptr<const mav_msgs::msgs::CmdActor> CmdActorPtr;

  class GZ_PLUGIN_VISIBLE ActorPlugin5 : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin5()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(NULL) {}

    virtual ~ActorPlugin5();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    private: void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    //private: void HandleObstacles(ignition::math::Vector3d &_pos);

    private:
    /// \brief Pointer to the update event connection.
    event::ConnectionPtr update_connection_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;

    std::string namespace_;

    std::string frame_id_;
    std::string link_name_;

    transport::NodePtr node_handle_;
    transport::PublisherPtr actor_pose_pub_5;
    transport::SubscriberPtr cmd_pose_sub_5;
    transport::PublisherPtr cmd_pose_pub_5;

    mav_msgs::msgs::Actor pose_msg;
    mav_msgs::msgs::CmdActor cmd_pose_msg;

    /// \callback function of command actor pose
    void CmdPoseCallback(CmdActorPtr &cmd_msg);

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
    private: ignition::math::Vector3d velocity;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
    //private: double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    //private: double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    //private: double animationFactor = 1.0;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    private: physics::TrajectoryInfoPtr trajectoryInfo;
  };
}
#endif
