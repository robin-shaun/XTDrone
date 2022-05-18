/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
   Desc: GazeboVacuumGripper plugin for manipulating objects in Gazebo
 * Author: Kentaro Wada
 * Date: 7 Dec 2015
 */

#ifndef GAZEBO_ROS_VACUUM_GRIPPER_HH
#define GAZEBO_ROS_VACUUM_GRIPPER_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosVacuumGripper Plugin XML Reference and Example

  \brief Ros Vacuum Gripper Plugin.

  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
    - left_end_effector will be the power point
    - revolute type joint is necessary (fixed joint disappears on gazebo and plugin can't find the joint and link)
  \verbatim
    <link name="left_end_effector">
      <gravity>0</gravity>
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <box size="0.01 0.01 0.01"/>
        </geometry>
        <material name="transparent">
          <color rgba="0 0 0 0"/>
        </material>
      </visual>
      <inertial>
        <origin rpy="0 0 0" xyz="0.000000 0.000000 0.000000"/>
        <mass value="0.0001"/>
        <inertia ixx="1e-08" ixy="0" ixz="0" iyy="1e-08" iyz="0" izz="1e-08"/>
      </inertial>
    </link>
    <joint name="left_end_joint" type="revolute">
      <parent link="left_wrist" />
      <child link="left_end_effector" />
      <origin rpy="0 0 0" xyz="0.08 0 .44" />
      <limit effort="30" velocity="1.0" lower="0" upper="0" />
    </joint>
    <gazebo>
      <plugin name="gazebo_ros_vacuum_gripper" filename="libgazebo_ros_vacuum_gripper.so">
        <robotNamespace>/robot/left_vacuum_gripper</robotNamespace>
        <bodyName>left_end_effector</bodyName>
        <topicName>grasping</topicName>
      </plugin>
    </gazebo>
  \endverbatim


\{
*/


class GazeboRosVacuumGripper : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosVacuumGripper();

  /// \brief Destructor
  public: virtual ~GazeboRosVacuumGripper();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  private: bool OnServiceCallback(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res);
  private: bool OffServiceCallback(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res);

  private: bool status_;

  private: physics::ModelPtr parent_;

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;
  private: ros::Publisher pub_;
  private: ros::ServiceServer srv1_;
  private: ros::ServiceServer srv2_;

  /// \brief ROS Wrench topic name inputs
  private: std::string topic_name_;
  private: std::string service_name_;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

  /// \brief: keep track of number of connections
  private: int connect_count_;
  private: void Connect();
  private: void Disconnect();
};
/** \} */
/// @}
}
#endif
