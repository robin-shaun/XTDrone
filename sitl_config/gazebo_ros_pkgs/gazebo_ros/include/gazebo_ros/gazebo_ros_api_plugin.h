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
 * Desc: External interfaces for Gazebo
 * Author: Nate Koenig, John Hsu, Dave Coleman
 * Date: 25 Apr 2010
 */

#ifndef __GAZEBO_ROS_API_PLUGIN_HH__
#define __GAZEBO_ROS_API_PLUGIN_HH__

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include <tinyxml.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>

// Services
#include "std_srvs/Empty.h"

#include "gazebo_msgs/JointRequest.h"
#include "gazebo_msgs/BodyRequest.h"

#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/DeleteLight.h"

#include "gazebo_msgs/ApplyBodyWrench.h"

#include "gazebo_msgs/SetPhysicsProperties.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

#include "gazebo_msgs/SetJointProperties.h"

#include "gazebo_msgs/GetWorldProperties.h"

#include "gazebo_msgs/GetModelProperties.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"

#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/ApplyJointEffort.h"

#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/GetLinkState.h"

#include "gazebo_msgs/GetLightProperties.h"
#include "gazebo_msgs/SetLightProperties.h"

// Topics
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/PerformanceMetrics.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

// For model pose transform to set custom joint angles
#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <boost/shared_ptr.hpp>

// For physics dynamics reconfigure
#include <dynamic_reconfigure/server.h>
#include <gazebo_ros/PhysicsConfig.h>
#include "gazebo_msgs/SetPhysicsProperties.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

#include <boost/algorithm/string.hpp>

#ifndef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
#if (GAZEBO_MAJOR_VERSION == 11 && GAZEBO_MINOR_VERSION > 1) || \
    (GAZEBO_MAJOR_VERSION == 9 && GAZEBO_MINOR_VERSION > 14)
#define GAZEBO_ROS_HAS_PERFORMANCE_METRICS
#endif
#endif  // ifndef GAZEBO_ROS_HAS_PERFORMANCE_METRICS

namespace gazebo
{

/// \brief A plugin loaded within the gzserver on startup.
class GazeboRosApiPlugin : public SystemPlugin
{
public:
  /// \brief Constructor
  GazeboRosApiPlugin();

  /// \brief Destructor
  ~GazeboRosApiPlugin();

  /// \bried Detect if sig-int shutdown signal is recieved
  void shutdownSignal();

  /// \brief Gazebo-inherited load function
  ///
  /// Called before Gazebo is loaded. Must not block.
  /// Capitalized per Gazebo cpp style guidelines
  /// \param _argc Number of command line arguments.
  /// \param _argv Array of command line arguments.
  void Load(int argc, char** argv);

  /// \brief ros queue thread for this node
  void gazeboQueueThread();

  /// \brief advertise services
  void advertiseServices();

  /// \brief Callback for a subscriber connecting to LinkStates ros topic.
  void onLinkStatesConnect();

  /// \brief Callback for a subscriber connecting to ModelStates ros topic.
  void onModelStatesConnect();

  /// \brief Callback for a subscriber disconnecting from LinkStates ros topic.
  void onLinkStatesDisconnect();

  /// \brief Callback for a subscriber disconnecting from ModelStates ros topic.
  void onModelStatesDisconnect();

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  /// \brief Callback for a subscriber connecting to PerformanceMetrics ros topic.
  void onPerformanceMetricsConnect();

  /// \brief Callback for a subscriber disconnecting from PerformanceMetrics ros topic.
  void onPerformanceMetricsDisconnect();
#endif

  /// \brief Function for inserting a URDF into Gazebo from ROS Service Call
  bool spawnURDFModel(gazebo_msgs::SpawnModel::Request &req,
                      gazebo_msgs::SpawnModel::Response &res);

  /// \brief Both SDFs and converted URDFs get sent to this function for further manipulation from a ROS Service call
  bool spawnSDFModel(gazebo_msgs::SpawnModel::Request &req,
                     gazebo_msgs::SpawnModel::Response &res);

  /// \brief delete model given name
  bool deleteModel(gazebo_msgs::DeleteModel::Request &req,gazebo_msgs::DeleteModel::Response &res);

  /// \brief delete a given light by name
  bool deleteLight(gazebo_msgs::DeleteLight::Request &req,gazebo_msgs::DeleteLight::Response &res);

  /// \brief
  bool getModelState(gazebo_msgs::GetModelState::Request &req,gazebo_msgs::GetModelState::Response &res);

  /// \brief
  bool getModelProperties(gazebo_msgs::GetModelProperties::Request &req,gazebo_msgs::GetModelProperties::Response &res);

  /// \brief
  bool getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,gazebo_msgs::GetWorldProperties::Response &res);

  /// \brief
  bool getJointProperties(gazebo_msgs::GetJointProperties::Request &req,gazebo_msgs::GetJointProperties::Response &res);

  /// \brief
  bool getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,gazebo_msgs::GetLinkProperties::Response &res);

  /// \brief
  bool getLinkState(gazebo_msgs::GetLinkState::Request &req,gazebo_msgs::GetLinkState::Response &res);

  /// \brief
  bool getLightProperties(gazebo_msgs::GetLightProperties::Request &req,gazebo_msgs::GetLightProperties::Response &res);

  /// \brief
  bool setLightProperties(gazebo_msgs::SetLightProperties::Request &req,gazebo_msgs::SetLightProperties::Response &res);

  /// \brief
  bool setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,gazebo_msgs::SetLinkProperties::Response &res);

  /// \brief
  bool setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,gazebo_msgs::SetPhysicsProperties::Response &res);

  /// \brief
  bool getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,gazebo_msgs::GetPhysicsProperties::Response &res);

  /// \brief
  bool setJointProperties(gazebo_msgs::SetJointProperties::Request &req,gazebo_msgs::SetJointProperties::Response &res);

  /// \brief
  bool setModelState(gazebo_msgs::SetModelState::Request &req,gazebo_msgs::SetModelState::Response &res);

  /// \brief
  void updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state);

  /// \brief
  void updateModelStates(const gazebo_msgs::ModelStates::ConstPtr& model_states);
  //TSC

  /// \brief
  bool applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,gazebo_msgs::ApplyJointEffort::Response &res);

  /// \brief
  bool resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

  /// \brief
  bool resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

  /// \brief
  bool pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

  /// \brief
  bool unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);

  /// \brief
  bool clearJointForces(gazebo_msgs::JointRequest::Request &req,gazebo_msgs::JointRequest::Response &res);
  bool clearJointForces(std::string joint_name);

  /// \brief
  bool clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,gazebo_msgs::BodyRequest::Response &res);
  bool clearBodyWrenches(std::string body_name);

  /// \brief
  bool setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,gazebo_msgs::SetModelConfiguration::Response &res);

  /// \brief
  bool setLinkState(gazebo_msgs::SetLinkState::Request &req,gazebo_msgs::SetLinkState::Response &res);

  /// \brief
  void updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state);

  /// \brief
  bool applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,gazebo_msgs::ApplyBodyWrench::Response &res);

private:

  /// \brief
  void wrenchBodySchedulerSlot();

  /// \brief
  void forceJointSchedulerSlot();

  /// \brief Callback to WorldUpdateBegin that publishes /clock.
  /// If pub_clock_frequency_ <= 0 (default behavior), it publishes every time step.
  /// Otherwise, it attempts to publish at that frequency in Hz.
  void publishSimTime();

  /// \brief
  void publishLinkStates();

  /// \brief
  void publishModelStates();

  /// \brief
  void stripXmlDeclaration(std::string &model_xml);

  /// \brief Update the model name and pose of the SDF file before sending to Gazebo
  void updateSDFAttributes(TiXmlDocument &gazebo_model_xml,
                           const std::string &model_name,
                           const ignition::math::Vector3d &initial_xyz,
                           const ignition::math::Quaterniond &initial_q);

  /// \brief Update the model pose of the URDF file before sending to Gazebo
  void updateURDFModelPose(TiXmlDocument &gazebo_model_xml,
                           const ignition::math::Vector3d &initial_xyz,
                           const ignition::math::Quaterniond &initial_q);

  /// \brief Update the model name of the URDF file before sending to Gazebo
  void updateURDFName(TiXmlDocument &gazebo_model_xml, const std::string &model_name);

  /// \brief
  void walkChildAddRobotNamespace(TiXmlNode* model_xml);

  /// \brief
  bool spawnAndConform(TiXmlDocument &gazebo_model_xml, const std::string &model_name,
                       gazebo_msgs::SpawnModel::Response &res);

  /// \brief helper function for applyBodyWrench
  ///        shift wrench from reference frame to target frame
  void transformWrench(ignition::math::Vector3d &target_force, ignition::math::Vector3d &target_torque,
                       const ignition::math::Vector3d &reference_force,
                       const ignition::math::Vector3d &reference_torque,
                       const ignition::math::Pose3d &target_to_reference );

  /// \brief Used for the dynamic reconfigure callback function template
  void physicsReconfigureCallback(gazebo_ros::PhysicsConfig &config, uint32_t level);

  /// \brief waits for the rest of Gazebo to be ready before initializing the dynamic reconfigure services
  void physicsReconfigureThread();

  /// \brief Unused
  void onResponse(ConstResponsePtr &response);

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  /// \brief Subscriber callback for performance metrics. This will be send in the ROS network
  void onPerformanceMetrics(const boost::shared_ptr<gazebo::msgs::PerformanceMetrics const> &msg);
#endif
  /// \brief utility for checking if string is in URDF format
  bool isURDF(std::string model_xml);

  /// \brief utility for checking if string is in SDF format
  bool isSDF(std::string model_xml);

  /// \brief Connect to Gazebo via its plugin interface, get a pointer to the world, start events
  void loadGazeboRosApiPlugin(std::string world_name);

  /// \brief convert xml to Pose
  ignition::math::Pose3d parsePose(const std::string &str);

  /// \brief convert xml to Pose
  ignition::math::Vector3d parseVector3(const std::string &str);

  // track if the desconstructor event needs to occur
  bool plugin_loaded_;

  // detect if sigint event occurs
  bool stop_;
  gazebo::event::ConnectionPtr sigint_event_;

  std::string robot_namespace_;

  gazebo::transport::NodePtr gazebonode_;
  gazebo::transport::SubscriberPtr stat_sub_;
  gazebo::transport::PublisherPtr factory_pub_;
  gazebo::transport::PublisherPtr factory_light_pub_;
  gazebo::transport::PublisherPtr light_modify_pub_;
  gazebo::transport::SubscriberPtr performance_metric_sub_;
  gazebo::transport::PublisherPtr request_pub_;
  gazebo::transport::SubscriberPtr response_sub_;

  boost::shared_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue gazebo_queue_;
  boost::shared_ptr<boost::thread> gazebo_callback_queue_thread_;

  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr wrench_update_event_;
  gazebo::event::ConnectionPtr force_update_event_;
  gazebo::event::ConnectionPtr time_update_event_;
  gazebo::event::ConnectionPtr pub_link_states_event_;
  gazebo::event::ConnectionPtr pub_model_states_event_;
  gazebo::event::ConnectionPtr load_gazebo_ros_api_plugin_event_;

  ros::ServiceServer spawn_sdf_model_service_;
  ros::ServiceServer spawn_urdf_model_service_;
  ros::ServiceServer delete_model_service_;
  ros::ServiceServer delete_light_service_;
  ros::ServiceServer get_model_state_service_;
  ros::ServiceServer get_model_properties_service_;
  ros::ServiceServer get_world_properties_service_;
  ros::ServiceServer get_joint_properties_service_;
  ros::ServiceServer get_link_properties_service_;
  ros::ServiceServer get_link_state_service_;
  ros::ServiceServer get_light_properties_service_;
  ros::ServiceServer set_light_properties_service_;
  ros::ServiceServer set_link_properties_service_;
  ros::ServiceServer set_physics_properties_service_;
  ros::ServiceServer get_physics_properties_service_;
  ros::ServiceServer apply_body_wrench_service_;
  ros::ServiceServer set_joint_properties_service_;
  ros::ServiceServer set_model_state_service_;
  ros::ServiceServer apply_joint_effort_service_;
  ros::ServiceServer set_model_configuration_service_;
  ros::ServiceServer set_link_state_service_;
  ros::ServiceServer reset_simulation_service_;
  ros::ServiceServer reset_world_service_;
  ros::ServiceServer pause_physics_service_;
  ros::ServiceServer unpause_physics_service_;
  ros::ServiceServer clear_joint_forces_service_;
  ros::ServiceServer clear_body_wrenches_service_;
  ros::Subscriber    set_link_state_topic_;
  ros::Subscriber    set_model_state_topic_;
  ros::Subscriber    set_model_states_topic_;
  // TSC
  ros::Publisher     pub_link_states_;
  ros::Publisher     pub_model_states_;
  ros::Publisher     pub_performance_metrics_;
  int                pub_link_states_connection_count_;
  int                pub_model_states_connection_count_;
  int                pub_performance_metrics_connection_count_;

  // ROS comm
  boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;

  // physics dynamic reconfigure
  boost::shared_ptr<boost::thread> physics_reconfigure_thread_;
  bool physics_reconfigure_initialized_;
  ros::ServiceClient physics_reconfigure_set_client_;
  ros::ServiceClient physics_reconfigure_get_client_;
  boost::shared_ptr< dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig> > physics_reconfigure_srv_;
  dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig>::CallbackType physics_reconfigure_callback_;

  ros::Publisher     pub_clock_;
  int pub_clock_frequency_;
  gazebo::common::Time last_pub_clock_time_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  boost::mutex lock_;

  bool world_created_;

  class WrenchBodyJob
  {
  public:
    gazebo::physics::LinkPtr body;
    ignition::math::Vector3d force;
    ignition::math::Vector3d torque;
    ros::Time start_time;
    ros::Duration duration;
  };

  class ForceJointJob
  {
  public:
    gazebo::physics::JointPtr joint;
    double force; // should this be a array?
    ros::Time start_time;
    ros::Duration duration;
  };

  std::vector<GazeboRosApiPlugin::WrenchBodyJob*> wrench_body_jobs_;
  std::vector<GazeboRosApiPlugin::ForceJointJob*> force_joint_jobs_;

  /// \brief index counters to count the accesses on models via GetModelState
  std::map<std::string, unsigned int> access_count_get_model_state_;

  /// \brief enable the communication of gazebo information using ROS service/topics
  bool enable_ros_network_;
};
}
#endif
