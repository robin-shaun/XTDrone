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
 * Desc: Force Torque Sensor Plugin
 * Author: Francisco Suarez-Ruiz
 * Date: 5 June 2014
 */

#ifndef GAZEBO_ROS_FT_HH
#define GAZEBO_ROS_FT_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosFTSensor Plugin XML Reference and Example

\brief Ros Gazebo Ros Force/Torque Sensor Plugin.
This is a model plugin which broadcasts geometry_msgs/WrenchStamped messages
with measured force and torque on a specified joint.
The wrench is reported in the joint CHILD link frame and the measure direction
is child-to-parent link.

Example Usage:

\verbatim
<!-- Enable the Joint Feedback -->
<gazebo reference="JOINT_NAME">
<provideFeedback>true</provideFeedback>
</gazebo>
<!-- The ft_sensor plugin -->
<gazebo>
<plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
<updateRate>100.0</updateRate>
<topicName>ft_sensor_topic</topicName>
<jointName>JOINT_NAME</jointName>
</plugin>
</gazebo>
\endverbatim
\{
*/


/// \brief GazeboRosFT controller
/// This is a controller that simulates a 6 dof force sensor
class GazeboRosFT : public ModelPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity must be a Model
  public: GazeboRosFT();

  /// \brief Destructor
  public: virtual ~GazeboRosFT();

  /// \brief Load the controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Gaussian noise
  private: double gaussian_noise_;

  /// \brief Gaussian noise generator
  private: double GaussianKernel(double mu, double sigma);

  /// \brief A pointer to the Gazebo joint
  private: physics::JointPtr joint_;

  /// \brief A pointer to the Gazebo parent link
  private: physics::LinkPtr parent_link_;

  /// \brief A pointer to the Gazebo child link
  private: physics::LinkPtr child_link_;

  /// \brief A pointer to the Gazebo model
  private: physics::ModelPtr model_;

  /// \brief A pointer to the Gazebo world
  private: physics::WorldPtr world_;

  /// \brief A pointer to the ROS node. A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief ROS WrenchStamped message
  private: geometry_msgs::WrenchStamped wrench_msg_;

  /// \brief store bodyname
  private: std::string joint_name_;

  /// \brief ROS WrenchStamped topic name
  private: std::string topic_name_;

  /// \brief ROS frame transform name to use in the image message header.
  /// FIXME: extract link name directly?
  private: std::string frame_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock_;

  /// \brief save last_time
  private: common::Time last_time_;

  // rate control
  private: double update_rate_;

  /// \brief: keep track of number of connections
  private: int ft_connect_count_;
  private: void FTConnect();
  private: void FTDisconnect();

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

};

/** \} */
/// @}


}

#endif
