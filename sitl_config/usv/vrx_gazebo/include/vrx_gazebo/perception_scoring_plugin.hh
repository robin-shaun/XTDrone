/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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


/* Note - this code was originally derived from the ARIAC 
 * PopulationPlugin https://bitbucket.org/osrf/ariac/src/master/osrf_gear/include/osrf_gear/PopulationPlugin.hh
*/

#ifndef VRX_GAZEBO_PERCEPTION_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_PERCEPTION_SCORING_PLUGIN_HH_

#include <geographic_msgs/GeoPoseStamped.h>
#include <ros/ros.h>
#include <vector>
#include <memory>
#include <string>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/scoring_plugin.hh"


/// \brief Class to store information about each object to be populated.
class PerceptionObject
{
  /// \brief Simulation time in which the object should be spawned.
  public: double time;

  /// \brief amount of time in which the object should be spawned.
  public: double duration;

  /// \brief PerceptionObject type.
  public: std::string type;

  /// \brief PerceptionObject type.
  public: std::string name;

  /// \brief Pose in which the object should be placed in wam-v's frame.
  public: ignition::math::Pose3d trialPose;

  /// \brief Pose in which the object should be placed in global frame.
  private: ignition::math::Pose3d origPose;

  /// \brief ModelPtr to the model that this object is representing
  public: gazebo::physics::EntityPtr modelPtr;

  /// \brief bool to tell weather or not the object is open for attempts
  public: bool active = false;

  /// \brief error associated with the guess of a moel
  public: double error = -1.0;

  /// \brief constructor of perception object
  public: PerceptionObject(const double& _time,
                 const double& _duration,
                 const std::string& _type,
                 const std::string& _name,
                 const ignition::math::Pose3d& _trialPose,
                 const gazebo::physics::WorldPtr _world);

  /// \brief set the error of this boject if this object is active
  ///   and this is the lowest seen error
  public: void SetError(const double& _error);

  /// \brief move the object to where it is supposed to be relative to the frame
  /// \brief of the robot and make it active
  public: void StartTrial(const gazebo::physics::EntityPtr& _frame);

  /// \brief move the object back to its original location and make inactive
  public: void EndTrial();

  /// \return a string summarizing this object
  public: std::string Str();
};


/// \brief A plugin that allows models to be spawned at a given location in
/// a specific simulation time and then takes care of scoring correct
/// identification and localization of the objects.
///
/// The plugin accepts the following SDF parameters:
///
/// <object_sequence>: Contains the list of objects to be populated. An object
///                    should be declared as an <object> element with the
///                    following parameters:
///                      <time> Simulation time to be spawned.
///                      <type> Model.
///                      <name> Landmark name.
///                      <pose> Initial object pose.
///
/// <loop_forever>: Optional parameter. If true, all objects will be spawned
/// as a circular buffer. After spawning the last element of the collection,
/// the first one will be inserted.
///
/// <frame>: Optional parameter. If present, the poses of the objects will be
/// in the frame of this link/model. Otherwise the world frame is used.
///
/// <robot_namespace>: Optional parameter.  If present, specifies ROS namespace.
///
/// <landmark_topic>: Optional parameter.  Specify the topic to which the
///   plugin subscribes for receiving identification and localization msgs.
///   Default is "/vrx/perception/landmark"
///
/// <duration>: Optional parameter. Specify the time an object sticks around.
///   defaults to 5
///
/// Here's an example of a valid SDF:
///
/// <plugin filename="libperception_scoring_plugin.so"
///         name="perception_scoring_plugin">
///   <vehicle>wamv</vehicle>
///   <task_name>perception</task_name>
///   <initial_state_duration>1</initial_state_duration>
///   <ready_state_duration>1</ready_state_duration>
///   <running_state_duration>300</running_state_duration>
///
///   <!-- Parameters for PopulationPlugin -->
///   <loop_forever>false</loop_forever>
///   <frame>wamv</frame>
///   <object_sequence>
///     <object>
///       <time>10.0</time>
///       <type>surmark950410</type>
///       <name>red_0</name>
///       <pose>6 0 1 0 0 0</pose>
///     </object>
///     <object>
///       <time>10.0</time>
///       <type>surmark950400</type>
///       <name>green_0</name>
///       <pose>6 6 1 0 0 0</pose>
///     </object>
///   </object_sequence>
/// </plugin>
class PerceptionScoringPlugin : public ScoringPlugin
{
  /// \brief Constructor.
  public: PerceptionScoringPlugin();

  /// \brief Destructor.
  public: virtual ~PerceptionScoringPlugin();

  // Documentation inherited.
  public: virtual void Load(gazebo::physics::WorldPtr _world,
                            sdf::ElementPtr _sdf);

  /// \brief Update the plugin.
  protected: void OnUpdate();

  private: void OnAttempt(
    const geographic_msgs::GeoPoseStamped::ConstPtr &_msg);

  /// \brief Restart the object population list
  private: void Restart();

  // Documentation inherited.
  private: void OnRunning() override;

  // Documentation inherited.
  private: void ReleaseVehicle() override;

  private: int attemptBal = 0;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic where the object id/pose is received.
  private: std::string objectTopic;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief ROS subscriber
  private: ros::Subscriber objectSub;

  /// \brief World pointer.
  public: gazebo::physics::WorldPtr world;

  /// \brief SDF pointer.
  public: sdf::ElementPtr sdf;

  /// \brief Collection of objects to be spawned.
  public: std::vector<PerceptionObject> objects;

  /// \brief Connection event.
  public: gazebo::event::ConnectionPtr connection;

  /// \brief The time specified in the object is relative to this time.
  public: gazebo::common::Time startTime;

  /// \brief When true, "objects" will be repopulated when the object queue
  /// is empty, creating an infinite supply of objects.
  public: bool loopForever = false;

  /// \brief Link/model name for the object poses use as their frame of
  /// reference
  public: std::string frameName = std::string();

  /// \brief Link/model that the object poses use as their frame of reference.
  public: gazebo::physics::EntityPtr frame;

  /// \brief Last time (sim time) that the plugin was updated.
  public: gazebo::common::Time lastUpdateTime;

  /// \ brief count of how many objects have been despawned
  private: int objectsDespawned =0;
};

#endif
