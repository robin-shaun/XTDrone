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
#ifndef GAZEBO_ROS_IMU_HH
#define GAZEBO_ROS_IMU_HH

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRosIMU : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRosIMU();

    /// \brief Destructor
    public: virtual ~GazeboRosIMU();

    /// \brief Load the controller
    /// \param node XML config node
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    /// \brief The parent World
    private: physics::WorldPtr world_;

    /// \brief The link referred to by this plugin
    private: physics::LinkPtr link;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<sensor_msgs::Imu>::Ptr pub_Queue;

    /// \brief ros message
    private: sensor_msgs::Imu imu_msg_;

    /// \brief store link name
    private: std::string link_name_;

    /// \brief store frame name
    private: std::string frame_name_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief allow specifying constant xyz and rpy offsets
    private: ignition::math::Pose3d offset_;

    /// \brief A mutex to lock access to fields
    /// that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief save last_time
    private: common::Time last_time_;
    private: ignition::math::Vector3d last_vpos_;
    private: ignition::math::Vector3d last_veul_;
    private: ignition::math::Vector3d apos_;
    private: ignition::math::Vector3d aeul_;

    // rate control
    private: double update_rate_;

    /// \brief: keep initial pose to offset orientation in imu message
    private: ignition::math::Pose3d initial_pose_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu, double sigma);

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    /// \brief call back when using service
    private: bool ServiceCallback(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    private: ros::ServiceServer srv_;
    private: std::string service_name_;

    private: ros::CallbackQueue imu_queue_;
    private: void IMUQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;
  };
}
#endif
