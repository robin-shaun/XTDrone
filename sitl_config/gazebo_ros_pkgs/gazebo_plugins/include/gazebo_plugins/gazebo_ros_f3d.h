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
 * Desc: 3D Applied Force Feedback Interface
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef GAZEBO_ROS_F3D_HH
#define GAZEBO_ROS_F3D_HH

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

/// \brief GazeboRosF3D controller
/// This is a controller that simulates a 6 dof force sensor
class GazeboRosF3D : public ModelPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity must be a Model
  public: GazeboRosF3D();

  /// \brief Destructor
  public: virtual ~GazeboRosF3D();

  /// \brief Load the controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  private: physics::WorldPtr world_;

  /// \brief A pointer to the Gazebo Body
  private: physics::LinkPtr link_;


  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief ROS WrenchStamped message
  private: geometry_msgs::WrenchStamped wrench_msg_;

  /// \brief store bodyname
  private: std::string link_name_;

  /// \brief ROS WrenchStamped topic name
  private: std::string topic_name_;

  /// \brief ROS frame transform name to use in the image message header.
  ///        This should be simply map since the returned info is in Gazebo Global Frame.
  private: std::string frame_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock_;

  /// \brief: keep track of number of connections
  private: int f3d_connect_count_;
  private: void F3DConnect();
  private: void F3DDisconnect();

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

