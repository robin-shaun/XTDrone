/*
 * Copyright 2013 Open Source Robotics Foundation
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

/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig, John Hsu, Dave Coleman
 * Date: Jun 10 2013
 */

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo_ros/gazebo_ros_api_plugin.h>
#include <chrono>
#include <thread>

namespace gazebo
{

GazeboRosApiPlugin::GazeboRosApiPlugin() :
  plugin_loaded_(false),
  stop_(false),
  pub_link_states_connection_count_(0),
  pub_model_states_connection_count_(0),
  pub_performance_metrics_connection_count_(0),
  physics_reconfigure_initialized_(false),
  pub_clock_frequency_(0),
  world_created_(false),
  enable_ros_network_(true)
{
  robot_namespace_.clear();
}

GazeboRosApiPlugin::~GazeboRosApiPlugin()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","GazeboRosApiPlugin Deconstructor start");

  // Unload the sigint event
  sigint_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","After sigint_event unload");

  // Don't attempt to unload this plugin if it was never loaded in the Load() function
  if(!plugin_loaded_)
  {
    ROS_DEBUG_STREAM_NAMED("api_plugin","Deconstructor skipped because never loaded");
    return;
  }

  // Disconnect slots
  load_gazebo_ros_api_plugin_event_.reset();
  wrench_update_event_.reset();
  force_update_event_.reset();
  time_update_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Slots disconnected");

  if (pub_link_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    pub_link_states_event_.reset();
  if (pub_model_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    pub_model_states_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Disconnected World Updates");

  // Stop the multi threaded ROS spinner
  async_ros_spin_->stop();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Async ROS Spin Stopped");

  // Shutdown the ROS node
  nh_->shutdown();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Node Handle Shutdown");

  // Shutdown ROS queue
  gazebo_callback_queue_thread_->join();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Callback Queue Joined");

  // Physics Dynamic Reconfigure
  if (physics_reconfigure_thread_)
  {
    physics_reconfigure_thread_->join();
    ROS_DEBUG_STREAM_NAMED("api_plugin","Physics reconfigure joined");
  }

  // Delete Force and Wrench Jobs
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();)
  {
    delete (*iter);
    iter = force_joint_jobs_.erase(iter);
  }
  force_joint_jobs_.clear();
  ROS_DEBUG_STREAM_NAMED("api_plugin","ForceJointJobs deleted");
  for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
  {
    delete (*iter);
    iter = wrench_body_jobs_.erase(iter);
  }
  wrench_body_jobs_.clear();
  lock_.unlock();
  ROS_DEBUG_STREAM_NAMED("api_plugin","WrenchBodyJobs deleted");

  ROS_DEBUG_STREAM_NAMED("api_plugin","Unloaded");
}

void GazeboRosApiPlugin::shutdownSignal()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","shutdownSignal() recieved");
  stop_ = true;
}

void GazeboRosApiPlugin::Load(int argc, char** argv)
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","Load");

  // connect to sigint event
  sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboRosApiPlugin::shutdownSignal,this));

  // setup ros related
  if (!ros::isInitialized())
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
  else
    ROS_ERROR_NAMED("api_plugin", "Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

  // check if the ros master is available - required
  while(!ros::master::check())
  {
    ROS_WARN_STREAM_NAMED("api_plugin","No ROS master - start roscore to continue...");
    // wait 0.5 second
    // can't use ROS Time here b/c node handle is not yet initialized
    std::this_thread::sleep_for(std::chrono::microseconds(500*1000));

    if(stop_)
    {
      ROS_WARN_STREAM_NAMED("api_plugin","Canceled loading Gazebo ROS API plugin by sigint event");
      return;
    }
  }

  nh_.reset(new ros::NodeHandle("~")); // advertise topics and services in this node's namespace

  // Built-in multi-threaded ROS spinning
  async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
  async_ros_spin_->start();

  /// \brief setup custom callback queue
  gazebo_callback_queue_thread_.reset(new boost::thread( &GazeboRosApiPlugin::gazeboQueueThread, this) );

  // below needs the world to be created first
  load_gazebo_ros_api_plugin_event_ = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboRosApiPlugin::loadGazeboRosApiPlugin,this,_1));

  nh_->getParam("enable_ros_network", enable_ros_network_);

  if (enable_ros_network_)
  {
    /// \brief start a thread for the physics dynamic reconfigure node
    physics_reconfigure_thread_.reset(new boost::thread(boost::bind(&GazeboRosApiPlugin::physicsReconfigureThread, this)));
  }

  plugin_loaded_ = true;
  ROS_INFO_NAMED("api_plugin", "Finished loading Gazebo ROS API Plugin.");
}

void GazeboRosApiPlugin::loadGazeboRosApiPlugin(std::string world_name)
{
  // make sure things are only called once
  lock_.lock();
  if (world_created_)
  {
    lock_.unlock();
    return;
  }

  // set flag to true and load this plugin
  world_created_ = true;
  lock_.unlock();

  world_ = gazebo::physics::get_world(world_name);
  if (!world_)
  {
    //ROS_ERROR_NAMED("api_plugin", "world name: [%s]",world->Name().c_str());
    // connect helper function to signal for scheduling torque/forces, etc
    ROS_FATAL_NAMED("api_plugin", "cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
    return;
  }

  gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebonode_->Init(world_name);
  factory_pub_ = gazebonode_->Advertise<gazebo::msgs::Factory>("~/factory");
  factory_light_pub_ = gazebonode_->Advertise<gazebo::msgs::Light>("~/factory/light");
  light_modify_pub_ = gazebonode_->Advertise<gazebo::msgs::Light>("~/light/modify");
  request_pub_ = gazebonode_->Advertise<gazebo::msgs::Request>("~/request");
  response_sub_ = gazebonode_->Subscribe("~/response",&GazeboRosApiPlugin::onResponse, this);
  // reset topic connection counts
  pub_link_states_connection_count_ = 0;
  pub_model_states_connection_count_ = 0;
  pub_performance_metrics_connection_count_ = 0;

  // Manage clock for simulated ros time
  pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 10);

  /// \brief advertise all services
  if (enable_ros_network_)
    advertiseServices();

  // set param for use_sim_time if not set by user already
  if(!(nh_->hasParam("/use_sim_time")))
    nh_->setParam("/use_sim_time", true);

  nh_->getParam("pub_clock_frequency", pub_clock_frequency_);
#if GAZEBO_MAJOR_VERSION >= 8
  last_pub_clock_time_ = world_->SimTime();
#else
  last_pub_clock_time_ = world_->GetSimTime();
#endif

  // hooks for applying forces, publishing simtime on /clock
  wrench_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::wrenchBodySchedulerSlot,this));
  force_update_event_  = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::forceJointSchedulerSlot,this));
  time_update_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishSimTime,this));
}

void GazeboRosApiPlugin::onResponse(ConstResponsePtr &response)
{

}
#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
void GazeboRosApiPlugin::onPerformanceMetrics(const boost::shared_ptr<gazebo::msgs::PerformanceMetrics const> &msg)
{
  gazebo_msgs::PerformanceMetrics msg_ros;
  msg_ros.header.stamp = ros::Time::now();
  msg_ros.real_time_factor = msg->real_time_factor();
  for (auto sensor: msg->sensor())
  {
    gazebo_msgs::SensorPerformanceMetric sensor_msgs;
    sensor_msgs.sim_update_rate = sensor.sim_update_rate();
    sensor_msgs.real_update_rate = sensor.real_update_rate();
    sensor_msgs.name = sensor.name();

    if (sensor.has_fps())
    {
      sensor_msgs.fps = sensor.fps();
    }
    else{
      sensor_msgs.fps = -1;
    }

    msg_ros.sensors.push_back(sensor_msgs);
  }

  pub_performance_metrics_.publish(msg_ros);
}
#endif

void GazeboRosApiPlugin::gazeboQueueThread()
{
  static const double timeout = 0.001;
  while (nh_->ok())
  {
    gazebo_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosApiPlugin::advertiseServices()
{
  if (! enable_ros_network_)
  {
    ROS_INFO_NAMED("api_plugin", "ROS gazebo topics/services are disabled");
    return;
  }

  // Advertise spawn services on the custom queue
  std::string spawn_sdf_model_service_name("spawn_sdf_model");
  ros::AdvertiseServiceOptions spawn_sdf_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_sdf_model_service_name,
                                                                  boost::bind(&GazeboRosApiPlugin::spawnSDFModel,this,_1,_2),
                                                                  ros::VoidPtr(), &gazebo_queue_);
  spawn_sdf_model_service_ = nh_->advertiseService(spawn_sdf_model_aso);

  // Advertise spawn services on the custom queue
  std::string spawn_urdf_model_service_name("spawn_urdf_model");
  ros::AdvertiseServiceOptions spawn_urdf_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_urdf_model_service_name,
                                                                  boost::bind(&GazeboRosApiPlugin::spawnURDFModel,this,_1,_2),
                                                                  ros::VoidPtr(), &gazebo_queue_);
  spawn_urdf_model_service_ = nh_->advertiseService(spawn_urdf_model_aso);

  // Advertise delete services on the custom queue
  std::string delete_model_service_name("delete_model");
  ros::AdvertiseServiceOptions delete_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteModel>(
                                                                   delete_model_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::deleteModel,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  delete_model_service_ = nh_->advertiseService(delete_aso);

  // Advertise delete service for lights on the custom queue
  std::string delete_light_service_name("delete_light");
  ros::AdvertiseServiceOptions delete_light_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteLight>(
                                                                   delete_light_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::deleteLight,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  delete_light_service_ = nh_->advertiseService(delete_light_aso);

  // Advertise more services on the custom queue
  std::string get_model_properties_service_name("get_model_properties");
  ros::AdvertiseServiceOptions get_model_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelProperties>(
                                                                          get_model_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getModelProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_model_properties_service_ = nh_->advertiseService(get_model_properties_aso);

  // Advertise more services on the custom queue
  std::string get_model_state_service_name("get_model_state");
  ros::AdvertiseServiceOptions get_model_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelState>(
                                                                     get_model_state_service_name,
                                                                     boost::bind(&GazeboRosApiPlugin::getModelState,this,_1,_2),
                                                                     ros::VoidPtr(), &gazebo_queue_);
  get_model_state_service_ = nh_->advertiseService(get_model_state_aso);

  // Advertise more services on the custom queue
  std::string get_world_properties_service_name("get_world_properties");
  ros::AdvertiseServiceOptions get_world_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetWorldProperties>(
                                                                          get_world_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getWorldProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_world_properties_service_ = nh_->advertiseService(get_world_properties_aso);

  // Advertise more services on the custom queue
  std::string get_joint_properties_service_name("get_joint_properties");
  ros::AdvertiseServiceOptions get_joint_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetJointProperties>(
                                                                          get_joint_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getJointProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_joint_properties_service_ = nh_->advertiseService(get_joint_properties_aso);

  // Advertise more services on the custom queue
  std::string get_link_properties_service_name("get_link_properties");
  ros::AdvertiseServiceOptions get_link_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkProperties>(
                                                                         get_link_properties_service_name,
                                                                         boost::bind(&GazeboRosApiPlugin::getLinkProperties,this,_1,_2),
                                                                         ros::VoidPtr(), &gazebo_queue_);
  get_link_properties_service_ = nh_->advertiseService(get_link_properties_aso);

  // Advertise more services on the custom queue
  std::string get_link_state_service_name("get_link_state");
  ros::AdvertiseServiceOptions get_link_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkState>(
                                                                    get_link_state_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::getLinkState,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  get_link_state_service_ = nh_->advertiseService(get_link_state_aso);

  // Advertise more services on the custom queue
  std::string get_light_properties_service_name("get_light_properties");
  ros::AdvertiseServiceOptions get_light_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLightProperties>(
                                                                          get_light_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getLightProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_light_properties_service_ = nh_->advertiseService(get_light_properties_aso);

  // Advertise more services on the custom queue
  std::string set_light_properties_service_name("set_light_properties");
  ros::AdvertiseServiceOptions set_light_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLightProperties>(
                                                                          set_light_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::setLightProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  set_light_properties_service_ = nh_->advertiseService(set_light_properties_aso);

  // Advertise more services on the custom queue
  std::string get_physics_properties_service_name("get_physics_properties");
  ros::AdvertiseServiceOptions get_physics_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetPhysicsProperties>(
                                                                            get_physics_properties_service_name,
                                                                            boost::bind(&GazeboRosApiPlugin::getPhysicsProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  get_physics_properties_service_ = nh_->advertiseService(get_physics_properties_aso);

  // publish complete link states in world frame
  ros::AdvertiseOptions pub_link_states_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::LinkStates>(
                                                           "link_states",10,
                                                           boost::bind(&GazeboRosApiPlugin::onLinkStatesConnect,this),
                                                           boost::bind(&GazeboRosApiPlugin::onLinkStatesDisconnect,this),
                                                           ros::VoidPtr(), &gazebo_queue_);
  pub_link_states_ = nh_->advertise(pub_link_states_ao);

  // publish complete model states in world frame
  ros::AdvertiseOptions pub_model_states_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::ModelStates>(
                                                            "model_states",10,
                                                            boost::bind(&GazeboRosApiPlugin::onModelStatesConnect,this),
                                                            boost::bind(&GazeboRosApiPlugin::onModelStatesDisconnect,this),
                                                            ros::VoidPtr(), &gazebo_queue_);
  pub_model_states_ = nh_->advertise(pub_model_states_ao);

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  // publish performance metrics
  ros::AdvertiseOptions pub_performance_metrics_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::PerformanceMetrics>(
                                                                   "performance_metrics",10,
                                                                   boost::bind(&GazeboRosApiPlugin::onPerformanceMetricsConnect,this),
                                                                   boost::bind(&GazeboRosApiPlugin::onPerformanceMetricsDisconnect,this),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  pub_performance_metrics_ = nh_->advertise(pub_performance_metrics_ao);
#endif

  // Advertise more services on the custom queue
  std::string set_link_properties_service_name("set_link_properties");
  ros::AdvertiseServiceOptions set_link_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkProperties>(
                                                                         set_link_properties_service_name,
                                                                         boost::bind(&GazeboRosApiPlugin::setLinkProperties,this,_1,_2),
                                                                         ros::VoidPtr(), &gazebo_queue_);
  set_link_properties_service_ = nh_->advertiseService(set_link_properties_aso);

  // Advertise more services on the custom queue
  std::string set_physics_properties_service_name("set_physics_properties");
  ros::AdvertiseServiceOptions set_physics_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetPhysicsProperties>(
                                                                            set_physics_properties_service_name,
                                                                            boost::bind(&GazeboRosApiPlugin::setPhysicsProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  set_physics_properties_service_ = nh_->advertiseService(set_physics_properties_aso);

  // Advertise more services on the custom queue
  std::string set_model_state_service_name("set_model_state");
  ros::AdvertiseServiceOptions set_model_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelState>(
                                                                     set_model_state_service_name,
                                                                     boost::bind(&GazeboRosApiPlugin::setModelState,this,_1,_2),
                                                                     ros::VoidPtr(), &gazebo_queue_);
  set_model_state_service_ = nh_->advertiseService(set_model_state_aso);

  // Advertise more services on the custom queue
  std::string set_model_configuration_service_name("set_model_configuration");
  ros::AdvertiseServiceOptions set_model_configuration_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelConfiguration>(
                                                                             set_model_configuration_service_name,
                                                                             boost::bind(&GazeboRosApiPlugin::setModelConfiguration,this,_1,_2),
                                                                             ros::VoidPtr(), &gazebo_queue_);
  set_model_configuration_service_ = nh_->advertiseService(set_model_configuration_aso);

  // Advertise more services on the custom queue
  std::string set_joint_properties_service_name("set_joint_properties");
  ros::AdvertiseServiceOptions set_joint_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetJointProperties>(
                                                                          set_joint_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::setJointProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  set_joint_properties_service_ = nh_->advertiseService(set_joint_properties_aso);

  // Advertise more services on the custom queue
  std::string set_link_state_service_name("set_link_state");
  ros::AdvertiseServiceOptions set_link_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkState>(
                                                                    set_link_state_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::setLinkState,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  set_link_state_service_ = nh_->advertiseService(set_link_state_aso);

  // Advertise topic on custom queue
  // topic callback version for set_link_state
  ros::SubscribeOptions link_state_so =
    ros::SubscribeOptions::create<gazebo_msgs::LinkState>(
                                                          "set_link_state",10,
                                                          boost::bind( &GazeboRosApiPlugin::updateLinkState,this,_1),
                                                          ros::VoidPtr(), &gazebo_queue_);
  set_link_state_topic_ = nh_->subscribe(link_state_so);

  // topic callback version for set_model_state
  ros::SubscribeOptions model_state_so =
    ros::SubscribeOptions::create<gazebo_msgs::ModelState>(
                                                           "set_model_state",10,
                                                           boost::bind( &GazeboRosApiPlugin::updateModelState,this,_1),
                                                           ros::VoidPtr(), &gazebo_queue_);
  set_model_state_topic_ = nh_->subscribe(model_state_so);

  // TSC
  // topic callback version for set_model_states
  ros::SubscribeOptions model_states_so =
  ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(
                                                           "set_model_states",10,
                                                           boost::bind( &GazeboRosApiPlugin::updateModelStates,this,_1),
                                                           ros::VoidPtr(), &gazebo_queue_);
  set_model_states_topic_ = nh_->subscribe(model_states_so);
  // TSC

  // Advertise more services on the custom queue
  std::string pause_physics_service_name("pause_physics");
  ros::AdvertiseServiceOptions pause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          pause_physics_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::pausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  pause_physics_service_ = nh_->advertiseService(pause_physics_aso);

  // Advertise more services on the custom queue
  std::string unpause_physics_service_name("unpause_physics");
  ros::AdvertiseServiceOptions unpause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          unpause_physics_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::unpausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  unpause_physics_service_ = nh_->advertiseService(unpause_physics_aso);

  // Advertise more services on the custom queue
  std::string apply_body_wrench_service_name("apply_body_wrench");
  ros::AdvertiseServiceOptions apply_body_wrench_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyBodyWrench>(
                                                                       apply_body_wrench_service_name,
                                                                       boost::bind(&GazeboRosApiPlugin::applyBodyWrench,this,_1,_2),
                                                                       ros::VoidPtr(), &gazebo_queue_);
  apply_body_wrench_service_ = nh_->advertiseService(apply_body_wrench_aso);

  // Advertise more services on the custom queue
  std::string apply_joint_effort_service_name("apply_joint_effort");
  ros::AdvertiseServiceOptions apply_joint_effort_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyJointEffort>(
                                                                        apply_joint_effort_service_name,
                                                                        boost::bind(&GazeboRosApiPlugin::applyJointEffort,this,_1,_2),
                                                                        ros::VoidPtr(), &gazebo_queue_);
  apply_joint_effort_service_ = nh_->advertiseService(apply_joint_effort_aso);

  // Advertise more services on the custom queue
  std::string clear_joint_forces_service_name("clear_joint_forces");
  ros::AdvertiseServiceOptions clear_joint_forces_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::JointRequest>(
                                                                    clear_joint_forces_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::clearJointForces,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  clear_joint_forces_service_ = nh_->advertiseService(clear_joint_forces_aso);

  // Advertise more services on the custom queue
  std::string clear_body_wrenches_service_name("clear_body_wrenches");
  ros::AdvertiseServiceOptions clear_body_wrenches_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::BodyRequest>(
                                                                   clear_body_wrenches_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::clearBodyWrenches,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  clear_body_wrenches_service_ = nh_->advertiseService(clear_body_wrenches_aso);

  // Advertise more services on the custom queue
  std::string reset_simulation_service_name("reset_simulation");
  ros::AdvertiseServiceOptions reset_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_simulation_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::resetSimulation,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  reset_simulation_service_ = nh_->advertiseService(reset_simulation_aso);

  // Advertise more services on the custom queue
  std::string reset_world_service_name("reset_world");
  ros::AdvertiseServiceOptions reset_world_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_world_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::resetWorld,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  reset_world_service_ = nh_->advertiseService(reset_world_aso);
}

void GazeboRosApiPlugin::onLinkStatesConnect()
{
  pub_link_states_connection_count_++;
  if (pub_link_states_connection_count_ == 1) // connect on first subscriber
    pub_link_states_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishLinkStates,this));
}

void GazeboRosApiPlugin::onModelStatesConnect()
{
  pub_model_states_connection_count_++;
  if (pub_model_states_connection_count_ == 1) // connect on first subscriber
    pub_model_states_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishModelStates,this));
}

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
void GazeboRosApiPlugin::onPerformanceMetricsConnect()
{
  pub_performance_metrics_connection_count_++;
  if (pub_performance_metrics_connection_count_ == 1) // connect on first subscriber
  {
    performance_metric_sub_ = gazebonode_->Subscribe("/gazebo/performance_metrics",
      &GazeboRosApiPlugin::onPerformanceMetrics, this);
  }
}

void GazeboRosApiPlugin::onPerformanceMetricsDisconnect()
{
  pub_performance_metrics_connection_count_--;
  if (pub_performance_metrics_connection_count_ <= 0) // disconnect with no subscribers
  {
    performance_metric_sub_.reset();
    if (pub_performance_metrics_connection_count_ < 0) // should not be possible
      ROS_ERROR_NAMED("api_plugin", "One too many disconnect from pub_performance_metrics_ in gazebo_ros.cpp? something weird");
  }
}
#endif

void GazeboRosApiPlugin::onLinkStatesDisconnect()
{
  pub_link_states_connection_count_--;
  if (pub_link_states_connection_count_ <= 0) // disconnect with no subscribers
  {
    pub_link_states_event_.reset();
    if (pub_link_states_connection_count_ < 0) // should not be possible
      ROS_ERROR_NAMED("api_plugin", "One too many disconnect from pub_link_states_ in gazebo_ros.cpp? something weird");
  }
}

void GazeboRosApiPlugin::onModelStatesDisconnect()
{
  pub_model_states_connection_count_--;
  if (pub_model_states_connection_count_ <= 0) // disconnect with no subscribers
  {
    pub_model_states_event_.reset();
    if (pub_model_states_connection_count_ < 0) // should not be possible
      ROS_ERROR_NAMED("api_plugin", "One too many disconnect from pub_model_states_ in gazebo_ros.cpp? something weird");
  }
}

bool GazeboRosApiPlugin::spawnURDFModel(gazebo_msgs::SpawnModel::Request &req,
                                        gazebo_msgs::SpawnModel::Response &res)
{
  // get namespace for the corresponding model plugins
  robot_namespace_ = req.robot_namespace;

  // incoming entity string
  std::string model_xml = req.model_xml;

  if (!isURDF(model_xml))
  {
    ROS_ERROR_NAMED("api_plugin", "SpawnModel: Failure - entity format is invalid.");
    res.success = false;
    res.status_message = "SpawnModel: Failure - entity format is invalid.";
    return false;
  }

  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
  /// @todo: does tinyxml have functionality for this?
  /// @todo: should gazebo take care of the declaration?
  {
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    size_t pos1 = model_xml.find(open_bracket,0);
    size_t pos2 = model_xml.find(close_bracket,0);
    if (pos1 != std::string::npos && pos2 != std::string::npos)
      model_xml.replace(pos1,pos2-pos1+2,std::string(""));
  }

  // Remove comments from URDF
  {
    std::string open_comment("<!--");
    std::string close_comment("-->");
    size_t pos1;
    while((pos1 = model_xml.find(open_comment,0)) != std::string::npos){
      size_t pos2 = model_xml.find(close_comment,0);
      if (pos2 != std::string::npos)
        model_xml.replace(pos1,pos2-pos1+3,std::string(""));
    }
  }

  // Now, replace package://xxx with the full path to the package
  {
    std::string package_prefix("package://");
    size_t pos1 = model_xml.find(package_prefix,0);
    while (pos1 != std::string::npos)
    {
      size_t pos2 = model_xml.find("/", pos1+10);
      //ROS_DEBUG_NAMED("api_plugin", " pos %d %d",(int)pos1, (int)pos2);
      if (pos2 == std::string::npos || pos1 >= pos2)
      {
        ROS_ERROR_NAMED("api_plugin", "Malformed package name?");
        break;
      }

      std::string package_name = model_xml.substr(pos1+10,pos2-pos1-10);
      //ROS_DEBUG_NAMED("api_plugin", "package name [%s]", package_name.c_str());
      std::string package_path = ros::package::getPath(package_name);
      if (package_path.empty())
      {
        ROS_FATAL_NAMED("api_plugin", "Package[%s] does not have a path",package_name.c_str());
        res.success = false;
        res.status_message = "urdf reference package name does not exist: " + package_name;
        return false;
      }
      ROS_DEBUG_ONCE_NAMED("api_plugin", "Package name [%s] has path [%s]", package_name.c_str(), package_path.c_str());

      model_xml.replace(pos1,(pos2-pos1),package_path);
      pos1 = model_xml.find(package_prefix, pos1);
    }
  }
  // ROS_DEBUG_NAMED("api_plugin", "Model XML\n\n%s\n\n ",model_xml.c_str());

  req.model_xml = model_xml;

  // Model is now considered convert to SDF
  return spawnSDFModel(req,res);
}

bool GazeboRosApiPlugin::spawnSDFModel(gazebo_msgs::SpawnModel::Request &req,
                                       gazebo_msgs::SpawnModel::Response &res)
{
  // incoming entity name
  std::string model_name = req.model_name;

  // get namespace for the corresponding model plugins
  robot_namespace_ = req.robot_namespace;

  // get initial pose of model
  ignition::math::Vector3d initial_xyz(req.initial_pose.position.x,req.initial_pose.position.y,req.initial_pose.position.z);
  // get initial roll pitch yaw (fixed frame transform)
  ignition::math::Quaterniond initial_q(req.initial_pose.orientation.w,req.initial_pose.orientation.x,req.initial_pose.orientation.y,req.initial_pose.orientation.z);

  // refernce frame for initial pose definition, modify initial pose if defined
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::EntityPtr frame = world_->EntityByName(req.reference_frame);
#else
  gazebo::physics::EntityPtr frame = world_->GetEntity(req.reference_frame);
#endif
  if (frame)
  {
    // convert to relative pose
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d frame_pose = frame->WorldPose();
#else
    ignition::math::Pose3d frame_pose = frame->GetWorldPose().Ign();
#endif
    initial_xyz = frame_pose.Pos() + frame_pose.Rot().RotateVector(initial_xyz);
    initial_q = frame_pose.Rot() * initial_q;
  }

  /// @todo: map is really wrong, need to use tf here somehow
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_DEBUG_NAMED("api_plugin", "SpawnModel: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    res.success = false;
    res.status_message = "SpawnModel: reference reference_frame not found, did you forget to scope the link by model name?";
    return true;
  }

  // incoming robot model string
  std::string model_xml = req.model_xml;

  // store resulting Gazebo Model XML to be sent to spawn queue
  // get incoming string containg either an URDF or a Gazebo Model XML
  // grab from parameter server if necessary convert to SDF if necessary
  stripXmlDeclaration(model_xml);

  // put string in TiXmlDocument for manipulation
  TiXmlDocument gazebo_model_xml;
  gazebo_model_xml.Parse(model_xml.c_str());

  // optional model manipulations: update initial pose && replace model name
  if (isSDF(model_xml))
  {
    updateSDFAttributes(gazebo_model_xml, model_name, initial_xyz, initial_q);

    // Walk recursively through the entire SDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!this->robot_namespace_.empty())
    {
      // Get root element for SDF
      // TODO: implement the spawning also with <light></light> and <model></model>
      TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("sdf");
      model_tixml = (!model_tixml) ?
          gazebo_model_xml.FirstChild("gazebo") : model_tixml;
      if (model_tixml)
      {
        walkChildAddRobotNamespace(model_tixml);
      }
      else
      {
        ROS_WARN_NAMED("api_plugin", "Unable to add robot namespace to xml");
      }
    }
  }
  else if (isURDF(model_xml))
  {
    updateURDFModelPose(gazebo_model_xml, initial_xyz, initial_q);
    updateURDFName(gazebo_model_xml, model_name);

    // Walk recursively through the entire URDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!this->robot_namespace_.empty())
    {
      // Get root element for URDF
      TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("robot");
      if (model_tixml)
      {
        walkChildAddRobotNamespace(model_tixml);
      }
      else
      {
        ROS_WARN_NAMED("api_plugin", "Unable to add robot namespace to xml");
      }
    }
  }
  else
  {
    ROS_ERROR_NAMED("api_plugin", "GazeboRosApiPlugin SpawnModel Failure: input xml format not recognized");
    res.success = false;
    res.status_message = "GazeboRosApiPlugin SpawnModel Failure: input model_xml not SDF or URDF, or cannot be converted to Gazebo compatible format.";
    return true;
  }

  // do spawning check if spawn worked, return response
  return spawnAndConform(gazebo_model_xml, model_name, res);
}

bool GazeboRosApiPlugin::deleteModel(gazebo_msgs::DeleteModel::Request &req,
                                     gazebo_msgs::DeleteModel::Response &res)
{
  // clear forces, etc for the body in question
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
#else
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
#endif
  if (!model)
  {
    ROS_ERROR_NAMED("api_plugin", "DeleteModel: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "DeleteModel: model does not exist";
    return true;
  }

  // delete wrench jobs on bodies
  for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
  {
    gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
    if (body)
    {
      // look for it in jobs, delete body wrench jobs
      clearBodyWrenches(body->GetScopedName());
    }
  }

  // delete force jobs on joints
  gazebo::physics::Joint_V joints = model->GetJoints();
  for (unsigned int i=0;i< joints.size(); i++)
  {
    // look for it in jobs, delete joint force jobs
    clearJointForces(joints[i]->GetName());
  }

  // send delete model request
  gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest("entity_delete",req.model_name);
  request_pub_->Publish(*msg,true);
  delete msg;

  ros::Duration model_spawn_timeout(60.0);
  ros::Time timeout = ros::Time::now() + model_spawn_timeout;
  // wait and verify that model is deleted
  while (true)
  {
    if (ros::Time::now() > timeout)
    {
      res.success = false;
      res.status_message = "DeleteModel: Model pushed to delete queue, but delete service timed out waiting for model to disappear from simulation";
      return true;
    }
    {
      //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
#if GAZEBO_MAJOR_VERSION >= 8
      if (!world_->ModelByName(req.model_name)) break;
#else
      if (!world_->GetModel(req.model_name)) break;
#endif
    }
    ROS_DEBUG_NAMED("api_plugin", "Waiting for model deletion (%s)",req.model_name.c_str());
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  // set result
  res.success = true;
  res.status_message = "DeleteModel: successfully deleted model";
  return true;
}

bool GazeboRosApiPlugin::deleteLight(gazebo_msgs::DeleteLight::Request &req,
                                     gazebo_msgs::DeleteLight::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);
#else
  gazebo::physics::LightPtr phy_light = world_->Light(req.light_name);
#endif

  if (phy_light == NULL)
  {
    res.success = false;
    res.status_message = "DeleteLight: Requested light " + req.light_name + " not found!";
  }
  else
  {
    gazebo::msgs::Request* msg = gazebo::msgs::CreateRequest("entity_delete", req.light_name);
    request_pub_->Publish(*msg, true);
    delete msg;

    res.success = false;

    for (int i = 0; i < 100; i++)
    {
#if GAZEBO_MAJOR_VERSION >= 8
      phy_light = world_->LightByName(req.light_name);
#else
      phy_light = world_->Light(req.light_name);
#endif
      if (phy_light == NULL)
      {
        res.success = true;
        res.status_message = "DeleteLight: " + req.light_name + " successfully deleted";
        return true;
      }
      // Check every 100ms
      std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
  }

  res.status_message = "DeleteLight: Timeout reached while removing light \"" + req.light_name
                       + "\"";

  return true;
}

bool GazeboRosApiPlugin::getModelState(gazebo_msgs::GetModelState::Request &req,
                                       gazebo_msgs::GetModelState::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
  gazebo::physics::EntityPtr frame = world_->EntityByName(req.relative_entity_name);
#else
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
  gazebo::physics::EntityPtr frame = world_->GetEntity(req.relative_entity_name);
#endif
  if (!model)
  {
    ROS_ERROR_NAMED("api_plugin", "GetModelState: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "GetModelState: model does not exist";
    return true;
  }
  else
  {
     /**
     * @brief creates a header for the result
     * @author Markus Bader markus.bader@tuwien.ac.at
     * @date 21th Nov 2014
     **/
    {
      std::map<std::string, unsigned int>::iterator it = access_count_get_model_state_.find(req.model_name);
      if(it == access_count_get_model_state_.end())
      {
        access_count_get_model_state_.insert( std::pair<std::string, unsigned int>(req.model_name, 1) );
        res.header.seq = 1;
      }
      else
      {
        it->second++;
        res.header.seq = it->second;
      }
      res.header.stamp = ros::Time::now();
      res.header.frame_id = req.relative_entity_name; /// @brief this is a redundant information
    }
    // get model pose
    // get model twist
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d      model_pose = model->WorldPose();
    ignition::math::Vector3d model_linear_vel  = model->WorldLinearVel();
    ignition::math::Vector3d model_angular_vel = model->WorldAngularVel();
#else
    ignition::math::Pose3d      model_pose = model->GetWorldPose().Ign();
    ignition::math::Vector3d model_linear_vel  = model->GetWorldLinearVel().Ign();
    ignition::math::Vector3d model_angular_vel = model->GetWorldAngularVel().Ign();
#endif
    ignition::math::Vector3d    model_pos = model_pose.Pos();
    ignition::math::Quaterniond model_rot = model_pose.Rot();



    if (frame)
    {
      // convert to relative pose, rates
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d frame_pose = frame->WorldPose();
      ignition::math::Vector3d frame_vpos = frame->WorldLinearVel(); // get velocity in gazebo frame
      ignition::math::Vector3d frame_veul = frame->WorldAngularVel(); // get velocity in gazebo frame
#else
      ignition::math::Pose3d frame_pose = frame->GetWorldPose().Ign();
      ignition::math::Vector3d frame_vpos = frame->GetWorldLinearVel().Ign(); // get velocity in gazebo frame
      ignition::math::Vector3d frame_veul = frame->GetWorldAngularVel().Ign(); // get velocity in gazebo frame
#endif
      ignition::math::Pose3d model_rel_pose = model_pose - frame_pose;
      model_pos = model_rel_pose.Pos();
      model_rot = model_rel_pose.Rot();

      model_linear_vel = frame_pose.Rot().RotateVectorReverse(model_linear_vel - frame_vpos);
      model_angular_vel = frame_pose.Rot().RotateVectorReverse(model_angular_vel - frame_veul);
    }
    /// @todo: FIXME map is really wrong, need to use tf here somehow
    else if (req.relative_entity_name == "" || req.relative_entity_name == "world" || req.relative_entity_name == "map" || req.relative_entity_name == "/map")
    {
      ROS_DEBUG_NAMED("api_plugin", "GetModelState: relative_entity_name is empty/world/map, using inertial frame");
    }
    else
    {
      res.success = false;
      res.status_message = "GetModelState: reference relative_entity_name not found, did you forget to scope the body by model name?";
      return true;
    }

    // fill in response
    res.pose.position.x = model_pos.X();
    res.pose.position.y = model_pos.Y();
    res.pose.position.z = model_pos.Z();
    res.pose.orientation.w = model_rot.W();
    res.pose.orientation.x = model_rot.X();
    res.pose.orientation.y = model_rot.Y();
    res.pose.orientation.z = model_rot.Z();

    res.twist.linear.x = model_linear_vel.X();
    res.twist.linear.y = model_linear_vel.Y();
    res.twist.linear.z = model_linear_vel.Z();
    res.twist.angular.x = model_angular_vel.X();
    res.twist.angular.y = model_angular_vel.Y();
    res.twist.angular.z = model_angular_vel.Z();

    res.success = true;
    res.status_message = "GetModelState: got properties";
    return true;
  }
  return true;
}

bool GazeboRosApiPlugin::getModelProperties(gazebo_msgs::GetModelProperties::Request &req,
                                            gazebo_msgs::GetModelProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
#else
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
#endif
  if (!model)
  {
    ROS_ERROR_NAMED("api_plugin", "GetModelProperties: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "GetModelProperties: model does not exist";
    return true;
  }
  else
  {
    // get model parent name
    gazebo::physics::ModelPtr parent_model = boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetParent());
    if (parent_model) res.parent_model_name = parent_model->GetName();

    // get list of child bodies, geoms
    res.body_names.clear();
    res.geom_names.clear();
    for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
      if (body)
      {
        res.body_names.push_back(body->GetName());
        // get list of geoms
        for (unsigned int j = 0; j < body->GetChildCount() ; j++)
        {
          gazebo::physics::CollisionPtr geom = boost::dynamic_pointer_cast<gazebo::physics::Collision>(body->GetChild(j));
          if (geom)
            res.geom_names.push_back(geom->GetName());
        }
      }
    }

    // get list of joints
    res.joint_names.clear();

    gazebo::physics::Joint_V joints = model->GetJoints();
    for (unsigned int i=0;i< joints.size(); i++)
      res.joint_names.push_back( joints[i]->GetName() );

    // get children model names
    res.child_model_names.clear();
    for (unsigned int j = 0; j < model->GetChildCount(); j++)
    {
      gazebo::physics::ModelPtr child_model = boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetChild(j));
      if (child_model)
        res.child_model_names.push_back(child_model->GetName() );
    }

    // is model static
    res.is_static = model->IsStatic();

    res.success = true;
    res.status_message = "GetModelProperties: got properties";
    return true;
  }
  return true;
}

bool GazeboRosApiPlugin::getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,
                                            gazebo_msgs::GetWorldProperties::Response &res)
{
  res.model_names.clear();
#if GAZEBO_MAJOR_VERSION >= 8
  res.sim_time = world_->SimTime().Double();
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
    res.model_names.push_back(world_->ModelByIndex(i)->GetName());
#else
  res.sim_time = world_->GetSimTime().Double();
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
    res.model_names.push_back(world_->GetModel(i)->GetName());
#endif
  // Note: currently rendering is always enabled in gazebo
  res.rendering_enabled = true; //world->GetRenderEngineEnabled();
  res.success = true;
  res.status_message = "GetWorldProperties: got properties";
  return true;
}

bool GazeboRosApiPlugin::getJointProperties(gazebo_msgs::GetJointProperties::Request &req,
                                            gazebo_msgs::GetJointProperties::Response &res)
{
  gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
#endif
    if (joint) break;
  }

  if (!joint)
  {
    res.success = false;
    res.status_message = "GetJointProperties: joint not found";
    return true;
  }
  else
  {
    /// @todo: FIXME
    res.type = res.REVOLUTE;

    res.damping.clear(); // to be added to gazebo
    //res.damping.push_back(joint->GetDamping(0));

    res.position.clear();
#if GAZEBO_MAJOR_VERSION >= 8
    res.position.push_back(joint->Position(0));
#else
    res.position.push_back(joint->GetAngle(0).Radian());
#endif

    res.rate.clear(); // use GetVelocity(i)
    res.rate.push_back(joint->GetVelocity(0));

    res.success = true;
    res.status_message = "GetJointProperties: got properties";
    return true;
  }
}

bool GazeboRosApiPlugin::getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,
                                           gazebo_msgs::GetLinkProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
#endif
  if (!body)
  {
    res.success = false;
    res.status_message = "GetLinkProperties: link not found, did you forget to scope the link by model name?";
    return true;
  }
  else
  {
    /// @todo: validate
    res.gravity_mode = body->GetGravityMode();

    gazebo::physics::InertialPtr inertia = body->GetInertial();

#if GAZEBO_MAJOR_VERSION >= 8
    res.mass = body->GetInertial()->Mass();

    res.ixx = inertia->IXX();
    res.iyy = inertia->IYY();
    res.izz = inertia->IZZ();
    res.ixy = inertia->IXY();
    res.ixz = inertia->IXZ();
    res.iyz = inertia->IYZ();

    ignition::math::Vector3d com = body->GetInertial()->CoG();
#else
    res.mass = body->GetInertial()->GetMass();

    res.ixx = inertia->GetIXX();
    res.iyy = inertia->GetIYY();
    res.izz = inertia->GetIZZ();
    res.ixy = inertia->GetIXY();
    res.ixz = inertia->GetIXZ();
    res.iyz = inertia->GetIYZ();

    ignition::math::Vector3d com = body->GetInertial()->GetCoG().Ign();
#endif
    res.com.position.x = com.X();
    res.com.position.y = com.Y();
    res.com.position.z = com.Z();
    res.com.orientation.x = 0; // @todo: gazebo do not support rotated inertia yet
    res.com.orientation.y = 0;
    res.com.orientation.z = 0;
    res.com.orientation.w = 1;

    res.success = true;
    res.status_message = "GetLinkProperties: got properties";
    return true;
  }
}

bool GazeboRosApiPlugin::getLinkState(gazebo_msgs::GetLinkState::Request &req,
                                      gazebo_msgs::GetLinkState::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
  gazebo::physics::EntityPtr frame = world_->EntityByName(req.reference_frame);
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
  gazebo::physics::EntityPtr frame = world_->GetEntity(req.reference_frame);
#endif

  if (!body)
  {
    res.success = false;
    res.status_message = "GetLinkState: link not found, did you forget to scope the link by model name?";
    return true;
  }

  // get body pose
  // Get inertial rates
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d body_pose = body->WorldPose();
  ignition::math::Vector3d body_vpos = body->WorldLinearVel(); // get velocity in gazebo frame
  ignition::math::Vector3d body_veul = body->WorldAngularVel(); // get velocity in gazebo frame
#else
  ignition::math::Pose3d body_pose = body->GetWorldPose().Ign();
  ignition::math::Vector3d body_vpos = body->GetWorldLinearVel().Ign(); // get velocity in gazebo frame
  ignition::math::Vector3d body_veul = body->GetWorldAngularVel().Ign(); // get velocity in gazebo frame
#endif

  if (frame)
  {
    // convert to relative pose, rates
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d frame_pose = frame->WorldPose();
    ignition::math::Vector3d frame_vpos = frame->WorldLinearVel(); // get velocity in gazebo frame
    ignition::math::Vector3d frame_veul = frame->WorldAngularVel(); // get velocity in gazebo frame
#else
    ignition::math::Pose3d frame_pose = frame->GetWorldPose().Ign();
    ignition::math::Vector3d frame_vpos = frame->GetWorldLinearVel().Ign(); // get velocity in gazebo frame
    ignition::math::Vector3d frame_veul = frame->GetWorldAngularVel().Ign(); // get velocity in gazebo frame
#endif
    body_pose = body_pose - frame_pose;

    body_vpos = frame_pose.Rot().RotateVectorReverse(body_vpos - frame_vpos);
    body_veul = frame_pose.Rot().RotateVectorReverse(body_veul - frame_veul);
  }
  /// @todo: FIXME map is really wrong, need to use tf here somehow
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_DEBUG_NAMED("api_plugin", "GetLinkState: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    res.success = false;
    res.status_message = "GetLinkState: reference reference_frame not found, did you forget to scope the link by model name?";
    return true;
  }

  res.link_state.link_name = req.link_name;
  res.link_state.pose.position.x = body_pose.Pos().X();
  res.link_state.pose.position.y = body_pose.Pos().Y();
  res.link_state.pose.position.z = body_pose.Pos().Z();
  res.link_state.pose.orientation.x = body_pose.Rot().X();
  res.link_state.pose.orientation.y = body_pose.Rot().Y();
  res.link_state.pose.orientation.z = body_pose.Rot().Z();
  res.link_state.pose.orientation.w = body_pose.Rot().W();
  res.link_state.twist.linear.x = body_vpos.X();
  res.link_state.twist.linear.y = body_vpos.Y();
  res.link_state.twist.linear.z = body_vpos.Z();
  res.link_state.twist.angular.x = body_veul.X();
  res.link_state.twist.angular.y = body_veul.Y();
  res.link_state.twist.angular.z = body_veul.Z();
  res.link_state.reference_frame = req.reference_frame;

  res.success = true;
  res.status_message = "GetLinkState: got state";
  return true;
}

bool GazeboRosApiPlugin::getLightProperties(gazebo_msgs::GetLightProperties::Request &req,
                                               gazebo_msgs::GetLightProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);
#else
  gazebo::physics::LightPtr phy_light = world_->Light(req.light_name);
#endif

  if (phy_light == NULL)
  {
      res.success = false;
      res.status_message = "getLightProperties: Requested light " + req.light_name + " not found!";
  }
  else
  {
    gazebo::msgs::Light light;
    phy_light->FillMsg(light);

    res.diffuse.r = light.diffuse().r();
    res.diffuse.g = light.diffuse().g();
    res.diffuse.b = light.diffuse().b();
    res.diffuse.a = light.diffuse().a();

    res.attenuation_constant = light.attenuation_constant();
    res.attenuation_linear = light.attenuation_linear();
    res.attenuation_quadratic = light.attenuation_quadratic();

    res.success = true;
  }

  return true;
}

bool GazeboRosApiPlugin::setLightProperties(gazebo_msgs::SetLightProperties::Request &req,
                                               gazebo_msgs::SetLightProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);
#else
  gazebo::physics::LightPtr phy_light = world_->Light(req.light_name);
#endif

  if (phy_light == NULL)
  {
    res.success = false;
    res.status_message = "setLightProperties: Requested light " + req.light_name + " not found!";
  }
  else
  {
    gazebo::msgs::Light light;

    phy_light->FillMsg(light);

    light.mutable_diffuse()->set_r(req.diffuse.r);
    light.mutable_diffuse()->set_g(req.diffuse.g);
    light.mutable_diffuse()->set_b(req.diffuse.b);
    light.mutable_diffuse()->set_a(req.diffuse.a);

    light.set_attenuation_constant(req.attenuation_constant);
    light.set_attenuation_linear(req.attenuation_linear);
    light.set_attenuation_quadratic(req.attenuation_quadratic);

    light_modify_pub_->Publish(light, true);

    res.success = true;
  }

  return true;
}

bool GazeboRosApiPlugin::setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,
                                           gazebo_msgs::SetLinkProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
#endif
  if (!body)
  {
    res.success = false;
    res.status_message = "SetLinkProperties: link not found, did you forget to scope the link by model name?";
    return true;
  }
  else
  {
    gazebo::physics::InertialPtr mass = body->GetInertial();
    // @todo: FIXME: add inertia matrix rotation to Gazebo
    // mass.SetInertiaRotation(ignition::math::Quaterniondion(req.com.orientation.w,res.com.orientation.x,req.com.orientation.y req.com.orientation.z));
    mass->SetCoG(ignition::math::Vector3d(req.com.position.x,req.com.position.y,req.com.position.z));
    mass->SetInertiaMatrix(req.ixx,req.iyy,req.izz,req.ixy,req.ixz,req.iyz);
    mass->SetMass(req.mass);
    body->SetGravityMode(req.gravity_mode);
    // @todo: mass change unverified
    res.success = true;
    res.status_message = "SetLinkProperties: properties set";
    return true;
  }
}

bool GazeboRosApiPlugin::setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,
                                              gazebo_msgs::SetPhysicsProperties::Response &res)
{
  // pause simulation if requested
  bool is_paused = world_->IsPaused();
  world_->SetPaused(true);
  world_->SetGravity(ignition::math::Vector3d(req.gravity.x,req.gravity.y,req.gravity.z));

  // supported updates
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::PhysicsEnginePtr pe = (world_->Physics());
#else
  gazebo::physics::PhysicsEnginePtr pe = (world_->GetPhysicsEngine());
#endif
  pe->SetMaxStepSize(req.time_step);
  pe->SetRealTimeUpdateRate(req.max_update_rate);

  if (pe->GetType() == "ode")
  {
    // stuff only works in ODE right now
    pe->SetAutoDisableFlag(req.ode_config.auto_disable_bodies);
    pe->SetParam("precon_iters", int(req.ode_config.sor_pgs_precon_iters));
    pe->SetParam("iters", int(req.ode_config.sor_pgs_iters));
    pe->SetParam("sor", req.ode_config.sor_pgs_w);
    pe->SetParam("cfm", req.ode_config.cfm);
    pe->SetParam("erp", req.ode_config.erp);
    pe->SetParam("contact_surface_layer",
        req.ode_config.contact_surface_layer);
    pe->SetParam("contact_max_correcting_vel",
        req.ode_config.contact_max_correcting_vel);
    pe->SetParam("max_contacts", int(req.ode_config.max_contacts));

    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "physics engine updated";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR_NAMED("api_plugin", "ROS set_physics_properties service call does not yet support physics engine [%s].", pe->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + pe->GetType() + "]: set_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,
                                              gazebo_msgs::GetPhysicsProperties::Response &res)
{
  // supported updates
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::PhysicsEnginePtr pe = (world_->Physics());
#else
  gazebo::physics::PhysicsEnginePtr pe = (world_->GetPhysicsEngine());
#endif
  res.time_step = pe->GetMaxStepSize();
  res.pause = world_->IsPaused();
  res.max_update_rate = pe->GetRealTimeUpdateRate();
  ignition::math::Vector3d gravity = world_->Gravity();
  res.gravity.x = gravity.X();
  res.gravity.y = gravity.Y();
  res.gravity.z = gravity.Z();

  // stuff only works in ODE right now
  if (pe->GetType() == "ode")
  {
    res.ode_config.auto_disable_bodies =
      pe->GetAutoDisableFlag();
    res.ode_config.sor_pgs_precon_iters = boost::any_cast<int>(
      pe->GetParam("precon_iters"));
    res.ode_config.sor_pgs_iters = boost::any_cast<int>(
        pe->GetParam("iters"));
    res.ode_config.sor_pgs_w = boost::any_cast<double>(
        pe->GetParam("sor"));
    res.ode_config.contact_surface_layer = boost::any_cast<double>(
      pe->GetParam("contact_surface_layer"));
    res.ode_config.contact_max_correcting_vel = boost::any_cast<double>(
      pe->GetParam("contact_max_correcting_vel"));
    res.ode_config.cfm = boost::any_cast<double>(
        pe->GetParam("cfm"));
    res.ode_config.erp = boost::any_cast<double>(
        pe->GetParam("erp"));
    res.ode_config.max_contacts = boost::any_cast<int>(
      pe->GetParam("max_contacts"));

    res.success = true;
    res.status_message = "GetPhysicsProperties: got properties";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR_NAMED("api_plugin", "ROS get_physics_properties service call does not yet support physics engine [%s].", pe->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + pe->GetType() + "]: get_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::setJointProperties(gazebo_msgs::SetJointProperties::Request &req,
                                            gazebo_msgs::SetJointProperties::Response &res)
{
  /// @todo: current settings only allows for setting of 1DOF joints (e.g. HingeJoint and SliderJoint) correctly.
  gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
#endif
    if (joint) break;
  }

  if (!joint)
  {
    res.success = false;
    res.status_message = "SetJointProperties: joint not found";
    return true;
  }
  else
  {
    for(unsigned int i=0;i< req.ode_joint_config.damping.size();i++)
      joint->SetDamping(i,req.ode_joint_config.damping[i]);
    for(unsigned int i=0;i< req.ode_joint_config.hiStop.size();i++)
      joint->SetParam("hi_stop",i,req.ode_joint_config.hiStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.loStop.size();i++)
      joint->SetParam("lo_stop",i,req.ode_joint_config.loStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.erp.size();i++)
      joint->SetParam("erp",i,req.ode_joint_config.erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.cfm.size();i++)
      joint->SetParam("cfm",i,req.ode_joint_config.cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_erp.size();i++)
      joint->SetParam("stop_erp",i,req.ode_joint_config.stop_erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_cfm.size();i++)
      joint->SetParam("stop_cfm",i,req.ode_joint_config.stop_cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fudge_factor.size();i++)
      joint->SetParam("fudge_factor",i,req.ode_joint_config.fudge_factor[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fmax.size();i++)
      joint->SetParam("fmax",i,req.ode_joint_config.fmax[i]);
    for(unsigned int i=0;i< req.ode_joint_config.vel.size();i++)
      joint->SetParam("vel",i,req.ode_joint_config.vel[i]);

    res.success = true;
    res.status_message = "SetJointProperties: properties set";
    return true;
  }
}

bool GazeboRosApiPlugin::setModelState(gazebo_msgs::SetModelState::Request &req,
                                       gazebo_msgs::SetModelState::Response &res)
{
  ignition::math::Vector3d target_pos(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);
  ignition::math::Quaterniond target_rot(req.model_state.pose.orientation.w,req.model_state.pose.orientation.x,req.model_state.pose.orientation.y,req.model_state.pose.orientation.z);
  target_rot.Normalize(); // eliminates invalid rotation (0, 0, 0, 0)
  ignition::math::Pose3d target_pose(target_pos,target_rot);
  ignition::math::Vector3d target_pos_dot(req.model_state.twist.linear.x,req.model_state.twist.linear.y,req.model_state.twist.linear.z);
  ignition::math::Vector3d target_rot_dot(req.model_state.twist.angular.x,req.model_state.twist.angular.y,req.model_state.twist.angular.z);

#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_state.model_name);
#else
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_state.model_name);
#endif
  if (!model)
  {
    ROS_ERROR_NAMED("api_plugin", "Updating ModelState: model [%s] does not exist",req.model_state.model_name.c_str());
    res.success = false;
    res.status_message = "SetModelState: model does not exist";
    return true;
  }
  else
  {
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::EntityPtr relative_entity = world_->EntityByName(req.model_state.reference_frame);
#else
    gazebo::physics::EntityPtr relative_entity = world_->GetEntity(req.model_state.reference_frame);
#endif
    if (relative_entity)
    {
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d  frame_pose = relative_entity->WorldPose(); // - myBody->GetCoMPose();
#else
      ignition::math::Pose3d  frame_pose = relative_entity->GetWorldPose().Ign(); // - myBody->GetCoMPose();
#endif

      target_pose = target_pose + frame_pose;

      // Velocities should be commanded in the requested reference
      // frame, so we need to translate them to the world frame
      target_pos_dot = frame_pose.Rot().RotateVector(target_pos_dot);
      target_rot_dot = frame_pose.Rot().RotateVector(target_rot_dot);
    }
    /// @todo: FIXME map is really wrong, need to use tf here somehow
    else if (req.model_state.reference_frame == "" || req.model_state.reference_frame == "world" || req.model_state.reference_frame == "map" || req.model_state.reference_frame == "/map" )
    {
      ROS_DEBUG_NAMED("api_plugin", "Updating ModelState: reference frame is empty/world/map, usig inertial frame");
    }
    else
    {
      ROS_ERROR_NAMED("api_plugin", "Updating ModelState: for model[%s], specified reference frame entity [%s] does not exist",
                req.model_state.model_name.c_str(),req.model_state.reference_frame.c_str());
      res.success = false;
      res.status_message = "SetModelState: specified reference frame entity does not exist";
      return true;
    }

    //ROS_ERROR_NAMED("api_plugin", "target state: %f %f %f",target_pose.Pos().X(),target_pose.Pos().Y(),target_pose.Pos().Z());
    bool is_paused = world_->IsPaused();
    world_->SetPaused(true);
    model->SetWorldPose(target_pose);
    world_->SetPaused(is_paused);
#if GAZEBO_MAJOR_VERSION >= 8
    //ignition::math::Pose3d p3d = model->WorldPose();
#else
    //ignition::math::Pose3d p3d = model->GetWorldPose().Ign();
#endif
    //ROS_ERROR_NAMED("api_plugin", "model updated state: %f %f %f",p3d.Pos().X(),p3d.Pos().Y(),p3d.Pos().Z());

    // set model velocity
    model->SetLinearVel(target_pos_dot);
    model->SetAngularVel(target_rot_dot);

    res.success = true;
    res.status_message = "SetModelState: set model state done";
    return true;
  }
}

void GazeboRosApiPlugin::updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state)
{
  gazebo_msgs::SetModelState::Response res;
  gazebo_msgs::SetModelState::Request req;
  req.model_state = *model_state;
  /*bool success =*/ setModelState(req,res);
}


//TSC
void GazeboRosApiPlugin::updateModelStates(const gazebo_msgs::ModelStates::ConstPtr& model_states)
{
  int model_num = 0;
  model_num = model_states->name.size();
  gazebo_msgs::SetModelState::Response res;
  gazebo_msgs::SetModelState::Request req;
  for(int index = 0;index < model_num;index++){
      req.model_state.model_name = model_states->name[index];
      req.model_state.pose = model_states->pose[index];
      req.model_state.twist = model_states->twist[index];
      req.model_state.reference_frame = "world";
  /*bool success =*/ setModelState(req,res);
  }
}


bool GazeboRosApiPlugin::applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,
                                          gazebo_msgs::ApplyJointEffort::Response &res)
{
  gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
#endif
    if (joint)
    {
      GazeboRosApiPlugin::ForceJointJob* fjj = new GazeboRosApiPlugin::ForceJointJob;
      fjj->joint = joint;
      fjj->force = req.effort;
      fjj->start_time = req.start_time;
#if GAZEBO_MAJOR_VERSION >= 8
      if (fjj->start_time < ros::Time(world_->SimTime().Double()))
        fjj->start_time = ros::Time(world_->SimTime().Double());
#else
      if (fjj->start_time < ros::Time(world_->GetSimTime().Double()))
        fjj->start_time = ros::Time(world_->GetSimTime().Double());
#endif
      fjj->duration = req.duration;
      lock_.lock();
      force_joint_jobs_.push_back(fjj);
      lock_.unlock();

      res.success = true;
      res.status_message = "ApplyJointEffort: effort set";
      return true;
    }
  }

  res.success = false;
  res.status_message = "ApplyJointEffort: joint not found";
  return true;
}

bool GazeboRosApiPlugin::resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->Reset();
  return true;
}

bool GazeboRosApiPlugin::resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->ResetEntities(gazebo::physics::Base::MODEL);
  return true;
}

bool GazeboRosApiPlugin::pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->SetPaused(true);
  return true;
}

bool GazeboRosApiPlugin::unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->SetPaused(false);
  return true;
}

bool GazeboRosApiPlugin::clearJointForces(gazebo_msgs::JointRequest::Request &req,
                                          gazebo_msgs::JointRequest::Response &res)
{
  return clearJointForces(req.joint_name);
}
bool GazeboRosApiPlugin::clearJointForces(std::string joint_name)
{
  bool search = true;
  lock_.lock();
  while(search)
  {
    search = false;
    for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();++iter)
    {
      if ((*iter)->joint->GetName() == joint_name)
      {
        // found one, search through again
        search = true;
        delete (*iter);
        force_joint_jobs_.erase(iter);
        break;
      }
    }
  }
  lock_.unlock();
  return true;
}

bool GazeboRosApiPlugin::clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,
                                           gazebo_msgs::BodyRequest::Response &res)
{
  return clearBodyWrenches(req.body_name);
}
bool GazeboRosApiPlugin::clearBodyWrenches(std::string body_name)
{
  bool search = true;
  lock_.lock();
  while(search)
  {
    search = false;
    for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();++iter)
    {
      //ROS_ERROR_NAMED("api_plugin", "search %s %s",(*iter)->body->GetScopedName().c_str(), body_name.c_str());
      if ((*iter)->body->GetScopedName() == body_name)
      {
        // found one, search through again
        search = true;
        delete (*iter);
        wrench_body_jobs_.erase(iter);
        break;
      }
    }
  }
  lock_.unlock();
  return true;
}

bool GazeboRosApiPlugin::setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,
                                               gazebo_msgs::SetModelConfiguration::Response &res)
{
  std::string gazebo_model_name = req.model_name;

  // search for model with name
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr gazebo_model = world_->ModelByName(req.model_name);
#else
  gazebo::physics::ModelPtr gazebo_model = world_->GetModel(req.model_name);
#endif
  if (!gazebo_model)
  {
    ROS_ERROR_NAMED("api_plugin", "SetModelConfiguration: model [%s] does not exist",gazebo_model_name.c_str());
    res.success = false;
    res.status_message = "SetModelConfiguration: model does not exist";
    return true;
  }

  if (req.joint_names.size() == req.joint_positions.size())
  {
    std::map<std::string, double> joint_position_map;
    for (unsigned int i = 0; i < req.joint_names.size(); i++)
    {
      joint_position_map[req.joint_names[i]] = req.joint_positions[i];
    }

    // make the service call to pause gazebo
    bool is_paused = world_->IsPaused();
    if (!is_paused) world_->SetPaused(true);

    gazebo_model->SetJointPositions(joint_position_map);

    // resume paused state before this call
    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "SetModelConfiguration: success";
    return true;
  }
  else
  {
    res.success = false;
    res.status_message = "SetModelConfiguration: joint name and position list have different lengths";
    return true;
  }
}

bool GazeboRosApiPlugin::setLinkState(gazebo_msgs::SetLinkState::Request &req,
                                      gazebo_msgs::SetLinkState::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_state.link_name));
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_state.reference_frame));
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_state.link_name));
  gazebo::physics::EntityPtr frame = world_->GetEntity(req.link_state.reference_frame);
#endif
  if (!body)
  {
    ROS_ERROR_NAMED("api_plugin", "Updating LinkState: link [%s] does not exist",req.link_state.link_name.c_str());
    res.success = false;
    res.status_message = "SetLinkState: link does not exist";
    return true;
  }

  /// @todo: FIXME map is really wrong, unless using tf here somehow
  // get reference frame (body/model(link)) pose and
  // transform target pose to absolute world frame
  ignition::math::Vector3d target_pos(req.link_state.pose.position.x,req.link_state.pose.position.y,req.link_state.pose.position.z);
  ignition::math::Quaterniond target_rot(req.link_state.pose.orientation.w,req.link_state.pose.orientation.x,req.link_state.pose.orientation.y,req.link_state.pose.orientation.z);
  ignition::math::Pose3d target_pose(target_pos,target_rot);
  ignition::math::Vector3d target_linear_vel(req.link_state.twist.linear.x,req.link_state.twist.linear.y,req.link_state.twist.linear.z);
  ignition::math::Vector3d target_angular_vel(req.link_state.twist.angular.x,req.link_state.twist.angular.y,req.link_state.twist.angular.z);

  if (frame)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d  frame_pose = frame->WorldPose(); // - myBody->GetCoMPose();
    ignition::math::Vector3d frame_linear_vel = frame->WorldLinearVel();
    ignition::math::Vector3d frame_angular_vel = frame->WorldAngularVel();
#else
    ignition::math::Pose3d  frame_pose = frame->GetWorldPose().Ign(); // - myBody->GetCoMPose();
    ignition::math::Vector3d frame_linear_vel = frame->GetWorldLinearVel().Ign();
    ignition::math::Vector3d frame_angular_vel = frame->GetWorldAngularVel().Ign();
#endif
    ignition::math::Vector3d frame_pos = frame_pose.Pos();
    ignition::math::Quaterniond frame_rot = frame_pose.Rot();

    //std::cout << " debug : " << frame->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
    target_pose = target_pose + frame_pose;

    target_linear_vel -= frame_linear_vel;
    target_angular_vel -= frame_angular_vel;
  }
  else if (req.link_state.reference_frame == "" || req.link_state.reference_frame == "world" || req.link_state.reference_frame == "map" || req.link_state.reference_frame == "/map")
  {
    ROS_INFO_NAMED("api_plugin", "Updating LinkState: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    ROS_ERROR_NAMED("api_plugin", "Updating LinkState: reference_frame is not a valid entity name");
    res.success = false;
    res.status_message = "SetLinkState: failed";
    return true;
  }

  //std::cout << " debug : " << target_pose << std::endl;
  //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());

  bool is_paused = world_->IsPaused();
  if (!is_paused) world_->SetPaused(true);
  body->SetWorldPose(target_pose);
  world_->SetPaused(is_paused);

  // set body velocity to desired twist
  body->SetLinearVel(target_linear_vel);
  body->SetAngularVel(target_angular_vel);

  res.success = true;
  res.status_message = "SetLinkState: success";
  return true;
}

void GazeboRosApiPlugin::updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state)
{
  gazebo_msgs::SetLinkState::Request req;
  gazebo_msgs::SetLinkState::Response res;
  req.link_state = *link_state;
  /*bool success = */ setLinkState(req,res);
}

void GazeboRosApiPlugin::transformWrench( ignition::math::Vector3d &target_force, ignition::math::Vector3d &target_torque,
                                          const ignition::math::Vector3d &reference_force,
                                          const ignition::math::Vector3d &reference_torque,
                                          const ignition::math::Pose3d &target_to_reference )
{
  // rotate force into target frame
  target_force = target_to_reference.Rot().RotateVector(reference_force);
  // rotate torque into target frame
  target_torque = target_to_reference.Rot().RotateVector(reference_torque);

  // target force is the refence force rotated by the target->reference transform
  target_torque = target_torque + target_to_reference.Pos().Cross(target_force);
}

bool GazeboRosApiPlugin::applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,
                                         gazebo_msgs::ApplyBodyWrench::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.body_name));
  gazebo::physics::EntityPtr frame = world_->EntityByName(req.reference_frame);
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.body_name));
  gazebo::physics::EntityPtr frame = world_->GetEntity(req.reference_frame);
#endif
  if (!body)
  {
    ROS_ERROR_NAMED("api_plugin", "ApplyBodyWrench: body [%s] does not exist",req.body_name.c_str());
    res.success = false;
    res.status_message = "ApplyBodyWrench: body does not exist";
    return true;
  }

  // target wrench
  ignition::math::Vector3d reference_force(req.wrench.force.x,req.wrench.force.y,req.wrench.force.z);
  ignition::math::Vector3d reference_torque(req.wrench.torque.x,req.wrench.torque.y,req.wrench.torque.z);
  ignition::math::Vector3d reference_point(req.reference_point.x,req.reference_point.y,req.reference_point.z);

  ignition::math::Vector3d target_force;
  ignition::math::Vector3d target_torque;

  /// shift wrench to body frame if a non-zero reference point is given
  ///   @todo: to be more general, should we make the reference point a reference pose?
  reference_torque = reference_torque + reference_point.Cross(reference_force);

  /// @todo: FIXME map is really wrong, need to use tf here somehow
  if (frame)
  {
    // get reference frame (body/model(body)) pose and
    // transform target pose to absolute world frame
    // @todo: need to modify wrench (target force and torque by frame)
    //        transform wrench from reference_point in reference_frame
    //        into the reference frame of the body
    //        first, translate by reference point to the body frame
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d framePose = frame->WorldPose();
    ignition::math::Pose3d bodyPose = body->WorldPose();
#else
    ignition::math::Pose3d framePose = frame->GetWorldPose().Ign();
    ignition::math::Pose3d bodyPose = body->GetWorldPose().Ign();
#endif
    ignition::math::Pose3d target_to_reference = framePose - bodyPose;
    ROS_DEBUG_NAMED("api_plugin", "reference frame for applied wrench: [%f %f %f, %f %f %f]-[%f %f %f, %f %f %f]=[%f %f %f, %f %f %f]",
              bodyPose.Pos().X(),
              bodyPose.Pos().Y(),
              bodyPose.Pos().Z(),
              bodyPose.Rot().Euler().X(),
              bodyPose.Rot().Euler().Y(),
              bodyPose.Rot().Euler().Z(),
              framePose.Pos().X(),
              framePose.Pos().Y(),
              framePose.Pos().Z(),
              framePose.Rot().Euler().X(),
              framePose.Rot().Euler().Y(),
              framePose.Rot().Euler().Z(),
              target_to_reference.Pos().X(),
              target_to_reference.Pos().Y(),
              target_to_reference.Pos().Z(),
              target_to_reference.Rot().Euler().X(),
              target_to_reference.Rot().Euler().Y(),
              target_to_reference.Rot().Euler().Z()
              );
    transformWrench(target_force, target_torque, reference_force, reference_torque, target_to_reference);
    ROS_ERROR_NAMED("api_plugin", "wrench defined as [%s]:[%f %f %f, %f %f %f] --> applied as [%s]:[%f %f %f, %f %f %f]",
              frame->GetName().c_str(),
              reference_force.X(),
              reference_force.Y(),
              reference_force.Z(),
              reference_torque.X(),
              reference_torque.Y(),
              reference_torque.Z(),
              body->GetName().c_str(),
              target_force.X(),
              target_force.Y(),
              target_force.Z(),
              target_torque.X(),
              target_torque.Y(),
              target_torque.Z()
              );

  }
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_INFO_NAMED("api_plugin", "ApplyBodyWrench: reference_frame is empty/world/map, using inertial frame, transferring from body relative to inertial frame");
    // FIXME: transfer to inertial frame
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d target_to_reference = body->WorldPose();
#else
    ignition::math::Pose3d target_to_reference = body->GetWorldPose().Ign();
#endif
    target_force = reference_force;
    target_torque = reference_torque;

  }
  else
  {
    ROS_ERROR_NAMED("api_plugin", "ApplyBodyWrench: reference_frame is not a valid entity name");
    res.success = false;
    res.status_message = "ApplyBodyWrench: reference_frame not found";
    return true;
  }

  // apply wrench
  // schedule a job to do below at appropriate times:
  // body->SetForce(force)
  // body->SetTorque(torque)
  GazeboRosApiPlugin::WrenchBodyJob* wej = new GazeboRosApiPlugin::WrenchBodyJob;
  wej->body = body;
  wej->force = target_force;
  wej->torque = target_torque;
  wej->start_time = req.start_time;
#if GAZEBO_MAJOR_VERSION >= 8
  if (wej->start_time < ros::Time(world_->SimTime().Double()))
    wej->start_time = ros::Time(world_->SimTime().Double());
#else
  if (wej->start_time < ros::Time(world_->GetSimTime().Double()))
    wej->start_time = ros::Time(world_->GetSimTime().Double());
#endif
  wej->duration = req.duration;
  lock_.lock();
  wrench_body_jobs_.push_back(wej);
  lock_.unlock();

  res.success = true;
  res.status_message = "";
  return true;
}

bool GazeboRosApiPlugin::isURDF(std::string model_xml)
{
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("robot"))
    return true;
  else
    return false;
}

bool GazeboRosApiPlugin::isSDF(std::string model_xml)
{
  // FIXME: very crude check
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("gazebo") ||
      doc_in.FirstChild("sdf")) // sdf
    return true;
  else
    return false;
}

void GazeboRosApiPlugin::wrenchBodySchedulerSlot()
{
  // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
  // boost::recursive_mutex::scoped_lock lock(*world->GetMDMutex());
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
  {
    // check times and apply wrench if necessary
#if GAZEBO_MAJOR_VERSION >= 8
    ros::Time simTime = ros::Time(world_->SimTime().Double());
#else
    ros::Time simTime = ros::Time(world_->GetSimTime().Double());
#endif
    if (simTime >= (*iter)->start_time)
      if (simTime <= (*iter)->start_time+(*iter)->duration ||
          (*iter)->duration.toSec() < 0.0)
      {
        if ((*iter)->body) // if body exists
        {
          (*iter)->body->SetForce((*iter)->force);
          (*iter)->body->SetTorque((*iter)->torque);
        }
        else
          (*iter)->duration.fromSec(0.0); // mark for delete
      }

    if (simTime > (*iter)->start_time+(*iter)->duration &&
        (*iter)->duration.toSec() >= 0.0)
    {
      // remove from queue once expires
      delete (*iter);
      iter = wrench_body_jobs_.erase(iter);
    }
    else
      ++iter;
  }
  lock_.unlock();
}

void GazeboRosApiPlugin::forceJointSchedulerSlot()
{
  // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
  // boost::recursive_mutex::scoped_lock lock(*world->GetMDMutex());
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();)
  {
    // check times and apply force if necessary
#if GAZEBO_MAJOR_VERSION >= 8
    ros::Time simTime = ros::Time(world_->SimTime().Double());
#else
    ros::Time simTime = ros::Time(world_->GetSimTime().Double());
#endif
    if (simTime >= (*iter)->start_time)
      if (simTime <= (*iter)->start_time+(*iter)->duration ||
          (*iter)->duration.toSec() < 0.0)
      {
        if ((*iter)->joint) // if joint exists
          (*iter)->joint->SetForce(0,(*iter)->force);
        else
          (*iter)->duration.fromSec(0.0); // mark for delete
      }

    if (simTime > (*iter)->start_time+(*iter)->duration &&
        (*iter)->duration.toSec() >= 0.0)
    {
      // remove from queue once expires
      iter = force_joint_jobs_.erase(iter);
    }
    else
      ++iter;
  }
  lock_.unlock();
}

void GazeboRosApiPlugin::publishSimTime()
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time sim_time = world_->SimTime();
#else
  gazebo::common::Time sim_time = world_->GetSimTime();
#endif
  if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).Double() < 1.0/pub_clock_frequency_)
    return;

#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time currentTime = world_->SimTime();
#else
  gazebo::common::Time currentTime = world_->GetSimTime();
#endif
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(currentTime.Double());
  //  publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}

void GazeboRosApiPlugin::publishLinkStates()
{
  gazebo_msgs::LinkStates link_states;

  // fill link_states
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);
#endif

    for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      if (body)
      {
        link_states.name.push_back(body->GetScopedName());
        geometry_msgs::Pose pose;
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d  body_pose = body->WorldPose(); // - myBody->GetCoMPose();
        ignition::math::Vector3d linear_vel  = body->WorldLinearVel();
        ignition::math::Vector3d angular_vel = body->WorldAngularVel();
#else
        ignition::math::Pose3d  body_pose = body->GetWorldPose().Ign(); // - myBody->GetCoMPose();
        ignition::math::Vector3d linear_vel  = body->GetWorldLinearVel().Ign();
        ignition::math::Vector3d angular_vel = body->GetWorldAngularVel().Ign();
#endif
        ignition::math::Vector3d pos = body_pose.Pos();
        ignition::math::Quaterniond rot = body_pose.Rot();
        pose.position.x = pos.X();
        pose.position.y = pos.Y();
        pose.position.z = pos.Z();
        pose.orientation.w = rot.W();
        pose.orientation.x = rot.X();
        pose.orientation.y = rot.Y();
        pose.orientation.z = rot.Z();
        link_states.pose.push_back(pose);
        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel.X();
        twist.linear.y = linear_vel.Y();
        twist.linear.z = linear_vel.Z();
        twist.angular.x = angular_vel.X();
        twist.angular.y = angular_vel.Y();
        twist.angular.z = angular_vel.Z();
        link_states.twist.push_back(twist);
      }
    }
  }

  pub_link_states_.publish(link_states);
}

void GazeboRosApiPlugin::publishModelStates()
{
  gazebo_msgs::ModelStates model_states;

  // fill model_states
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);
    ignition::math::Pose3d  model_pose = model->WorldPose(); // - myBody->GetCoMPose();
    ignition::math::Vector3d linear_vel  = model->WorldLinearVel();
    ignition::math::Vector3d angular_vel = model->WorldAngularVel();
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);
    ignition::math::Pose3d  model_pose = model->GetWorldPose().Ign(); // - myBody->GetCoMPose();
    ignition::math::Vector3d linear_vel  = model->GetWorldLinearVel().Ign();
    ignition::math::Vector3d angular_vel = model->GetWorldAngularVel().Ign();
#endif
    ignition::math::Vector3d pos = model_pose.Pos();
    ignition::math::Quaterniond rot = model_pose.Rot();
    geometry_msgs::Pose pose;
    pose.position.x = pos.X();
    pose.position.y = pos.Y();
    pose.position.z = pos.Z();
    pose.orientation.w = rot.W();
    pose.orientation.x = rot.X();
    pose.orientation.y = rot.Y();
    pose.orientation.z = rot.Z();
    model_states.pose.push_back(pose);
    model_states.name.push_back(model->GetName());
    geometry_msgs::Twist twist;
    twist.linear.x = linear_vel.X();
    twist.linear.y = linear_vel.Y();
    twist.linear.z = linear_vel.Z();
    twist.angular.x = angular_vel.X();
    twist.angular.y = angular_vel.Y();
    twist.angular.z = angular_vel.Z();
    model_states.twist.push_back(twist);
  }
  pub_model_states_.publish(model_states);
}

void GazeboRosApiPlugin::physicsReconfigureCallback(gazebo_ros::PhysicsConfig &config, uint32_t level)
{
  if (!physics_reconfigure_initialized_)
  {
    gazebo_msgs::GetPhysicsProperties srv;
    physics_reconfigure_get_client_.call(srv);

    config.time_step                   = srv.response.time_step;
    config.max_update_rate             = srv.response.max_update_rate;
    config.gravity_x                   = srv.response.gravity.x;
    config.gravity_y                   = srv.response.gravity.y;
    config.gravity_z                   = srv.response.gravity.z;
    config.auto_disable_bodies         = srv.response.ode_config.auto_disable_bodies;
    config.sor_pgs_precon_iters        = srv.response.ode_config.sor_pgs_precon_iters;
    config.sor_pgs_iters               = srv.response.ode_config.sor_pgs_iters;
    config.sor_pgs_rms_error_tol       = srv.response.ode_config.sor_pgs_rms_error_tol;
    config.sor_pgs_w                   = srv.response.ode_config.sor_pgs_w;
    config.contact_surface_layer       = srv.response.ode_config.contact_surface_layer;
    config.contact_max_correcting_vel  = srv.response.ode_config.contact_max_correcting_vel;
    config.cfm                         = srv.response.ode_config.cfm;
    config.erp                         = srv.response.ode_config.erp;
    config.max_contacts                = srv.response.ode_config.max_contacts;
    physics_reconfigure_initialized_ = true;
  }
  else
  {
    bool changed = false;
    gazebo_msgs::GetPhysicsProperties srv;
    physics_reconfigure_get_client_.call(srv);

    // check for changes
    if (config.time_step                      != srv.response.time_step)                                 changed = true;
    if (config.max_update_rate                != srv.response.max_update_rate)                           changed = true;
    if (config.gravity_x                      != srv.response.gravity.x)                                 changed = true;
    if (config.gravity_y                      != srv.response.gravity.y)                                 changed = true;
    if (config.gravity_z                      != srv.response.gravity.z)                                 changed = true;
    if (config.auto_disable_bodies            != srv.response.ode_config.auto_disable_bodies)            changed = true;
    if ((uint32_t)config.sor_pgs_precon_iters != srv.response.ode_config.sor_pgs_precon_iters)           changed = true;
    if ((uint32_t)config.sor_pgs_iters        != srv.response.ode_config.sor_pgs_iters)                  changed = true;
    if (config.sor_pgs_rms_error_tol          != srv.response.ode_config.sor_pgs_rms_error_tol)          changed = true;
    if (config.sor_pgs_w                      != srv.response.ode_config.sor_pgs_w)                      changed = true;
    if (config.contact_surface_layer          != srv.response.ode_config.contact_surface_layer)          changed = true;
    if (config.contact_max_correcting_vel     != srv.response.ode_config.contact_max_correcting_vel)     changed = true;
    if (config.cfm                            != srv.response.ode_config.cfm)                            changed = true;
    if (config.erp                            != srv.response.ode_config.erp)                            changed = true;
    if ((uint32_t)config.max_contacts         != srv.response.ode_config.max_contacts)                   changed = true;

    if (changed)
    {
      // pause simulation if requested
      gazebo_msgs::SetPhysicsProperties srv;
      srv.request.time_step                             = config.time_step                   ;
      srv.request.max_update_rate                       = config.max_update_rate             ;
      srv.request.gravity.x                             = config.gravity_x                   ;
      srv.request.gravity.y                             = config.gravity_y                   ;
      srv.request.gravity.z                             = config.gravity_z                   ;
      srv.request.ode_config.auto_disable_bodies        = config.auto_disable_bodies         ;
      srv.request.ode_config.sor_pgs_precon_iters       = config.sor_pgs_precon_iters        ;
      srv.request.ode_config.sor_pgs_iters              = config.sor_pgs_iters               ;
      srv.request.ode_config.sor_pgs_rms_error_tol      = config.sor_pgs_rms_error_tol       ;
      srv.request.ode_config.sor_pgs_w                  = config.sor_pgs_w                   ;
      srv.request.ode_config.contact_surface_layer      = config.contact_surface_layer       ;
      srv.request.ode_config.contact_max_correcting_vel = config.contact_max_correcting_vel  ;
      srv.request.ode_config.cfm                        = config.cfm                         ;
      srv.request.ode_config.erp                        = config.erp                         ;
      srv.request.ode_config.max_contacts               = config.max_contacts                ;
      physics_reconfigure_set_client_.call(srv);
      ROS_INFO_NAMED("api_plugin", "physics dynamics reconfigure update complete");
    }
    ROS_INFO_NAMED("api_plugin", "physics dynamics reconfigure complete");
  }
}

void GazeboRosApiPlugin::physicsReconfigureThread()
{
  physics_reconfigure_set_client_ = nh_->serviceClient<gazebo_msgs::SetPhysicsProperties>("set_physics_properties");
  physics_reconfigure_get_client_ = nh_->serviceClient<gazebo_msgs::GetPhysicsProperties>("get_physics_properties");

  // Wait until the rest of this plugin is loaded and the services are being offered
  physics_reconfigure_set_client_.waitForExistence();
  physics_reconfigure_get_client_.waitForExistence();

  physics_reconfigure_srv_.reset(new dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig>());

  physics_reconfigure_callback_ = boost::bind(&GazeboRosApiPlugin::physicsReconfigureCallback, this, _1, _2);
  physics_reconfigure_srv_->setCallback(physics_reconfigure_callback_);

  ROS_INFO_NAMED("api_plugin", "Physics dynamic reconfigure ready.");
}

void GazeboRosApiPlugin::stripXmlDeclaration(std::string &model_xml)
{
  // incoming robot model string is a string containing a Gazebo Model XML
  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
  /// @todo: does tinyxml have functionality for this?
  /// @todo: should gazebo take care of the declaration?
  std::string open_bracket("<?");
  std::string close_bracket("?>");
  size_t pos1 = model_xml.find(open_bracket,0);
  size_t pos2 = model_xml.find(close_bracket,0);
  if (pos1 != std::string::npos && pos2 != std::string::npos)
    model_xml.replace(pos1,pos2-pos1+2,std::string(""));
}

void GazeboRosApiPlugin::updateSDFAttributes(TiXmlDocument &gazebo_model_xml,
                                             const std::string &model_name,
                                             const ignition::math::Vector3d &initial_xyz,
                                             const ignition::math::Quaterniond &initial_q)
{
  // This function can handle both regular SDF files and <include> SDFs that are used with the
  // Gazebo Model Database

  TiXmlElement* pose_element; // This is used by both reguar and database SDFs

  // Check SDF for requires SDF element
  TiXmlElement* gazebo_tixml = gazebo_model_xml.FirstChildElement("sdf");
  if (!gazebo_tixml)
  {
    ROS_WARN_NAMED("api_plugin", "Could not find <sdf> element in sdf, so name and initial position cannot be applied");
    return;
  }

  // Check SDF for optional model element. May not have one
  TiXmlElement* model_tixml = gazebo_tixml->FirstChildElement("model");
  if (model_tixml)
  {
    // Update entity name
    if (model_tixml->Attribute("name") != NULL)
    {
      // removing old entity name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name",model_name);
  }
  else
  {
    // Check SDF for world element
    TiXmlElement* world_tixml = gazebo_tixml->FirstChildElement("world");
    if (!world_tixml)
    {
      ROS_WARN_NAMED("api_plugin", "Could not find <model> or <world> element in sdf, so name and initial position cannot be applied");
      return;
    }
    // If not <model> element, check SDF for required include element
    model_tixml = world_tixml->FirstChildElement("include");
    if (!model_tixml)
    {
      ROS_WARN_NAMED("api_plugin", "Could not find <include> element in sdf, so name and initial position cannot be applied");
      return;
    }

    // Check for name element
    TiXmlElement* name_tixml = model_tixml->FirstChildElement("name");
    if (!name_tixml)
    {
      // Create the name element
      name_tixml = new TiXmlElement("name");
      model_tixml->LinkEndChild(name_tixml);
    }

    // Set the text within the name element
    TiXmlText* text = new TiXmlText(model_name);
    name_tixml->LinkEndChild( text );
  }


  // Check for the pose element
  pose_element = model_tixml->FirstChildElement("pose");
  ignition::math::Pose3d model_pose;

  // Create the pose element if it doesn't exist
  // Remove it if it exists, since we are inserting a new one
  if (pose_element)
  {
    // save pose_element in ignition::math::Pose3d and remove child
    model_pose = this->parsePose(pose_element->GetText());
    model_tixml->RemoveChild(pose_element);
  }

  // Set and link the pose element after adding initial pose
  {
    // add pose_element Pose to initial pose
    ignition::math::Pose3d new_model_pose = model_pose + ignition::math::Pose3d(initial_xyz, initial_q);

    // Create the string of 6 numbers
    std::ostringstream pose_stream;
    ignition::math::Vector3d model_rpy = new_model_pose.Rot().Euler(); // convert to Euler angles for Gazebo XML
    pose_stream << new_model_pose.Pos().X() << " " << new_model_pose.Pos().Y() << " " << new_model_pose.Pos().Z() << " "
                << model_rpy.X() << " " << model_rpy.Y() << " " << model_rpy.Z();

    // Add value to pose element
    TiXmlText* text = new TiXmlText(pose_stream.str());
    TiXmlElement* new_pose_element = new TiXmlElement("pose");
    new_pose_element->LinkEndChild(text);
    model_tixml->LinkEndChild(new_pose_element);
  }
}

ignition::math::Pose3d GazeboRosApiPlugin::parsePose(const std::string &str)
{
  std::vector<std::string> pieces;
  std::vector<double> vals;

  boost::split(pieces, str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        sdferr << "xml key [" << str
          << "][" << i << "] value [" << pieces[i]
          << "] is not a valid double from a 3-tuple\n";
        return ignition::math::Pose3d();
      }
    }
  }

  if (vals.size() == 6)
    return ignition::math::Pose3d(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
  else
  {
    ROS_ERROR_NAMED("api_plugin", "Beware: failed to parse string [%s] as ignition::math::Pose3d, returning zeros.", str.c_str());
    return ignition::math::Pose3d();
  }
}

ignition::math::Vector3d GazeboRosApiPlugin::parseVector3(const std::string &str)
{
  std::vector<std::string> pieces;
  std::vector<double> vals;

  boost::split(pieces, str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        sdferr << "xml key [" << str
          << "][" << i << "] value [" << pieces[i]
          << "] is not a valid double from a 3-tuple\n";
        return ignition::math::Vector3d();
      }
    }
  }

  if (vals.size() == 3)
    return ignition::math::Vector3d(vals[0], vals[1], vals[2]);
  else
  {
    ROS_ERROR_NAMED("api_plugin", "Beware: failed to parse string [%s] as ignition::math::Vector3d, returning zeros.", str.c_str());
    return ignition::math::Vector3d();
  }
}

void GazeboRosApiPlugin::updateURDFModelPose(TiXmlDocument &gazebo_model_xml,
                                             const ignition::math::Vector3d &initial_xyz,
                                             const ignition::math::Quaterniond &initial_q)
{
  TiXmlElement* model_tixml = (gazebo_model_xml.FirstChildElement("robot"));
  if (model_tixml)
  {
    // replace initial pose of model
    // find first instance of xyz and rpy, replace with initial pose
    TiXmlElement* origin_key = model_tixml->FirstChildElement("origin");

    if (!origin_key)
    {
      origin_key = new TiXmlElement("origin");
      model_tixml->LinkEndChild(origin_key);
    }

    ignition::math::Vector3d xyz;
    ignition::math::Vector3d rpy;
    if (origin_key->Attribute("xyz"))
    {
      xyz = this->parseVector3(origin_key->Attribute("xyz"));
      origin_key->RemoveAttribute("xyz");
    }
    if (origin_key->Attribute("rpy"))
    {
      rpy = this->parseVector3(origin_key->Attribute("rpy"));
      origin_key->RemoveAttribute("rpy");
    }

    // add xyz, rpy to initial pose
    ignition::math::Pose3d model_pose = ignition::math::Pose3d(xyz, ignition::math::Quaterniond(rpy))
                                      + ignition::math::Pose3d(initial_xyz, initial_q);

    std::ostringstream xyz_stream;
    xyz_stream << model_pose.Pos().X() << " " << model_pose.Pos().Y() << " " << model_pose.Pos().Z();

    std::ostringstream rpy_stream;
    ignition::math::Vector3d model_rpy = model_pose.Rot().Euler(); // convert to Euler angles for Gazebo XML
    rpy_stream << model_rpy.X() << " " << model_rpy.Y() << " " << model_rpy.Z();

    origin_key->SetAttribute("xyz",xyz_stream.str());
    origin_key->SetAttribute("rpy",rpy_stream.str());
  }
  else
    ROS_WARN_NAMED("api_plugin", "Could not find <model> element in sdf, so name and initial position is not applied");
}

void GazeboRosApiPlugin::updateURDFName(TiXmlDocument &gazebo_model_xml, const std::string &model_name)
{
  TiXmlElement* model_tixml = gazebo_model_xml.FirstChildElement("robot");
  // replace model name if one is specified by the user
  if (model_tixml)
  {
    if (model_tixml->Attribute("name") != NULL)
    {
      // removing old model name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name",model_name);
  }
  else
    ROS_WARN_NAMED("api_plugin", "Could not find <robot> element in URDF, name not replaced");
}

void GazeboRosApiPlugin::walkChildAddRobotNamespace(TiXmlNode* model_xml)
{
  TiXmlNode* child = 0;
  child = model_xml->IterateChildren(child);
  while (child != NULL)
  {
    if (child->Type() == TiXmlNode::TINYXML_ELEMENT &&
        child->ValueStr().compare(std::string("plugin")) == 0)
    {
      if (child->FirstChildElement("robotNamespace") == NULL)
      {
        TiXmlElement* child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        while (child_elem)
        {
          child->ToElement()->RemoveChild(child_elem);
          child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        }
        TiXmlElement* key = new TiXmlElement("robotNamespace");
        TiXmlText* val = new TiXmlText(robot_namespace_);
        key->LinkEndChild(val);
        child->ToElement()->LinkEndChild(key);
      }
    }
    walkChildAddRobotNamespace(child);
    child = model_xml->IterateChildren(child);
  }
}

bool GazeboRosApiPlugin::spawnAndConform(TiXmlDocument &gazebo_model_xml, const std::string &model_name,
                                         gazebo_msgs::SpawnModel::Response &res)
{
  std::string entity_type = gazebo_model_xml.RootElement()->FirstChild()->Value();
  // Convert the entity type to lower case
  std::transform(entity_type.begin(), entity_type.end(), entity_type.begin(), ::tolower);

  bool isLight = (entity_type == "light");

  // push to factory iface
  std::ostringstream stream;
  stream << gazebo_model_xml;
  std::string gazebo_model_xml_string = stream.str();
  ROS_DEBUG_NAMED("api_plugin.xml", "Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());

  // publish to factory topic
  gazebo::msgs::Factory msg;
  gazebo::msgs::Init(msg, "spawn_model");
  msg.set_sdf( gazebo_model_xml_string );

  //ROS_ERROR_NAMED("api_plugin", "attempting to spawn model name [%s] [%s]", model_name.c_str(),gazebo_model_xml_string.c_str());

  // FIXME: should use entity_info or add lock to World::receiveMutex
  // looking for Model to see if it exists already
  gazebo::msgs::Request *entity_info_msg = gazebo::msgs::CreateRequest("entity_info", model_name);
  request_pub_->Publish(*entity_info_msg,true);
  delete entity_info_msg;
  // todo: should wait for response response_sub_, check to see that if _msg->response == "nonexistant"

#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr model = world_->ModelByName(model_name);
  gazebo::physics::LightPtr light = world_->LightByName(model_name);
#else
  gazebo::physics::ModelPtr model = world_->GetModel(model_name);
  gazebo::physics::LightPtr light = world_->Light(model_name);
#endif
  if ((isLight && light != NULL) || (model != NULL))
  {
    ROS_ERROR_NAMED("api_plugin", "SpawnModel: Failure - model name %s already exist.",model_name.c_str());
    res.success = false;
    res.status_message = "SpawnModel: Failure - entity already exists.";
    return true;
  }

  // for Gazebo 7 and up, use a different method to spawn lights
  if (isLight)
  {
    // Publish the light message to spawn the light (Gazebo 7 and up)
    sdf::SDF sdf_light;
    sdf_light.SetFromString(gazebo_model_xml_string);
    gazebo::msgs::Light msg = gazebo::msgs::LightFromSDF(sdf_light.Root()->GetElement("light"));
    msg.set_name(model_name);
    factory_light_pub_->Publish(msg);
  }
  else
  {
    // Publish the factory message
    factory_pub_->Publish(msg);
  }
  /// FIXME: should change publish to direct invocation World::LoadModel() and/or
  ///        change the poll for Model existence to common::Events based check.

  /// \brief poll and wait, verify that the model is spawned within Hardcoded 10 seconds
  ros::Duration model_spawn_timeout(10.0);
  ros::Time timeout = ros::Time::now() + model_spawn_timeout;

  while (ros::ok())
  {
    if (ros::Time::now() > timeout)
    {
      res.success = false;
      res.status_message = "SpawnModel: Entity pushed to spawn queue, but spawn service timed out waiting for entity to appear in simulation under the name " + model_name;
      return true;
    }

    {
      //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
#if GAZEBO_MAJOR_VERSION >= 8
      if ((isLight && world_->LightByName(model_name) != NULL)
          || (world_->ModelByName(model_name) != NULL))
#else
      if ((isLight && world_->Light(model_name) != NULL)
          || (world_->GetModel(model_name) != NULL))
#endif
        break;
    }

    ROS_DEBUG_STREAM_ONCE_NAMED("api_plugin","Waiting for " << timeout - ros::Time::now()
      << " for entity " << model_name << " to spawn");

    std::this_thread::sleep_for(std::chrono::microseconds(2000));
  }

  // set result
  res.success = true;
  res.status_message = "SpawnModel: Successfully spawned entity";
  return true;
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)
}
