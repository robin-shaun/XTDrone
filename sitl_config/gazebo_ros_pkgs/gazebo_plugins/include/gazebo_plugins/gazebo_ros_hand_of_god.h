/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 */

#ifndef GAZEBO_ROS_TEMPLATE_HH
#define GAZEBO_ROS_TEMPLATE_HH

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{

  class GazeboRosHandOfGod : public ModelPlugin
  {
  /// \brief Constructor
  public: GazeboRosHandOfGod();

  /// \brief Destructor
  public: virtual ~GazeboRosHandOfGod();

  /// \brief Load the controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief Update the controller
  protected: virtual void GazeboUpdate();

  /// Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
           boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
           boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
           boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
           physics::ModelPtr model_;
           physics::LinkPtr floating_link_;
           std::string link_name_;
           std::string robot_namespace_;
           std::string frame_id_;
           double kl_, ka_;
           double cl_, ca_;
  };

}

#endif

