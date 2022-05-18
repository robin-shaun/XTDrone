/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GAZEBO_ROS_WHEEL_SLIP_H
#define GAZEBO_ROS_WHEEL_SLIP_H

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

// dynamic reconfigure stuff
#include <gazebo_plugins/WheelSlipConfig.h>
#include <dynamic_reconfigure/server.h>

#include <gazebo/plugins/WheelSlipPlugin.hh>

namespace gazebo
{
/// \brief See the Gazebo documentation about the WheelSlipPlugin. This ROS
/// wrapper exposes two parameters via dynamic reconfigure:
///
///  1. slip_compliance_unitless_lateral
///      - Type: double
///      - Description: Unitless slip compliance (slip / friction) in the
///           lateral direction. This value is applied to all wheels declared
///           in the WheelSlipPlugin.
///
///  2. slip_compliance_unitless_longitudinal
///      - Type: double
///      - Description: Unitless slip compliance (slip / friction) in the
///           longitudinal direction. This value is applied to all wheels declared
///           in the WheelSlipPlugin.
class GazeboRosWheelSlip : public WheelSlipPlugin
{
    /// \brief Constructor
    public: GazeboRosWheelSlip();

    /// \brief Destructor
    public: virtual ~GazeboRosWheelSlip();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Custom callback queue thread
    private: void QueueThread();

    // Allow dynamic reconfiguration of wheel slip params
    private: void configCallback(
                    gazebo_plugins::WheelSlipConfig &config,
                    uint32_t level);

    /// \brief pointer to ros node
    private: ros::NodeHandle *rosnode_;

    /// \brief Dynamic reconfigure server.
    private: dynamic_reconfigure::Server<gazebo_plugins::WheelSlipConfig>
                    *dyn_srv_;

    /// \brief for setting ROS name space
    private: std::string robotNamespace_;
    private: ros::CallbackQueue queue_;
    private: boost::thread callbackQueueThread_;
};
}
#endif
