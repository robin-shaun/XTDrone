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

#ifndef VRX_GAZEBO_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_SCORING_PLUGIN_HH_

#include <ros/ros.h>
#include <gazebo/msgs/gz_string.pb.h>
#include <memory>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include <gazebo/transport/transport.hh>
#include "vrx_gazebo/Task.h"
#include "vrx_gazebo/Contact.h"

/// \brief A plugin that provides common functionality to any scoring plugin.
/// This plugin defines four different task states:
///
/// * Initial: The vehicle might be locked to the world via some joints with
/// different constraints. This state is used to stabilize the vehicle and let
/// it settle for a while.
/// * Ready: If the vehicle was locked, it will be released in this state. The
/// task hasn't started yet, but the vehicle can be controlled and prepared for
/// the start of the task.
/// * Running: The task has started. The clockwatch task starts too.
/// * Finished: The maximum allowed task time has been reached (time out) or
/// the task has been completed. Other plugins derived from the ScoringPlugin
/// class can call the Finish() method to trigger the completion of the task.
///
/// The plugin also exposes a pair of methods [Set]Score() for setting and
/// getting a score.
///
/// Derived plugins can also override the OnReady(), OnRunning(),
/// and OnFinished() to be notified when the task transitions into the "ready".
/// "running" or "finished" states respectively.
///
/// The plugin publishes task information on a ROS topic at 1Hz.
///
/// This plugin uses the following SDF parameters:
///
/// <vehicle>: Required parameter (string type) with the name of the main
/// vehicle to be under control during the task.
///
/// <task_name>: Required parameter specifying the task name (string type).
///
/// <task_info_topic>: Optional parameter (string type)
/// containing the ROS topic name to publish the task stats. The default
/// topic name is /vrx/task/info .
///
/// <contact_debug_topic>: Optional parameter (string type)
/// containing the ROS topic name to
/// publish every instant a collision with the wamv is happening.
/// Default is /vrx/debug/contact.
///
/// <per_plugin_exit_on_completion>: Specifies whether to shut down after
/// completion, for this specific plugin.
/// Different from environment variable VRX_EXIT_ON_COMPLETION, which is for
/// every plugin in the current shell. Environment variable overwrites this
/// parameter.
///
/// <initial_state_duration>: Optional parameter (double type) specifying the
/// amount of seconds that the plugin will be in the "initial" state.
///
/// <ready_state_duration>: Optional parameter (double type) specifying the
/// amount of seconds that the plugin will be in the "ready" state.
///
/// <running_state_duration>: Optional parameter (double type) specifying the
/// amount of maximum seconds that the plugin will be in the "running" state.
/// Note that this parameter specifies the maximum task time.
///
/// <collision_buffer>: Optional parameter (double type) specifying the
/// minimum amount of seconds between two collisions. If N collisions happen
/// within this time frame, only one will be counted.
///
/// <release_joints>: Optional element specifying the collection of joints that
/// should be dettached when transitioning to the "ready" state.
///
///   This block should contain at least one of these blocks:
///   <joint>: This block should contain a <name> element with the name of the
///   joint to release.
///
/// Here's an example:
/// <plugin name="scoring_plugin"
///         filename="libscoring_plugin.so">
///   <vehicle>wamv</vehicle>
///   <task_name>navigation_course</task_name>
///   <initial_state_duration>10</initial_state_duration>
///   <ready_state_duration>10</ready_state_duration>
///   <running_state_duration>30</running_state_duration>
///   <release_joints>
///     <joint>
///       <name>wamv_external_pivot_joint</name>
///     </joint>
///     <joint>
///       <name>wamv_external_riser</name>
///     </joint>
///   </release_joints>
/// </plugin>

class ScoringPlugin : public gazebo::WorldPlugin
{
  /// \brief Class constructor.
  public: ScoringPlugin();

  /// \brief Shutdown Gazebo and ROS.
  public: void Exit();

  // Documentation inherited.
  protected: void Load(gazebo::physics::WorldPtr _world,
                       sdf::ElementPtr _sdf);

  /// \brief Get the current score.
  /// \return The current score.
  protected: double Score() const;

  /// \brief Set the score.
  /// \param[in] _newScore The new score.
  protected: void SetScore(double _newScore);

  /// \brief Get the task name.
  /// \return Task name.
  protected: std::string TaskName() const;

  /// \brief Get the task state.
  /// \return Task state.
  protected: std::string TaskState() const;

  /// \brief Elapsed time in the running state.
  /// \return The elapsed time in the running state.
  protected: gazebo::common::Time ElapsedTime() const;

  /// \brief Remaining time in the running state.
  /// \return The remaining time in the running state.
  protected: gazebo::common::Time RemainingTime() const;

  /// \brief Finish the current task.
  /// This will set the "finished" flag in the task message to true.
  protected: void Finish();

  /// \brief Tries to release the vehicle in case is locked.
  protected: virtual void ReleaseVehicle();

  /// \brief Set the score in case of timeout
  protected: void SetTimeoutScore(double _timeoutScore);

  /// \brief Get the timeoutScore
  protected: double GetTimeoutScore();

  /// \brief Get running duration
  protected: double GetRunningStateDuration();

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief Update all time-related variables.
  private: void UpdateTime();

  /// \brief Update the state of the current task.
  private: void UpdateTaskState();

  /// \brief Update the task stats message.
  private: void UpdateTaskMessage();

  /// \brief Publish the task stats over a ROS topic.
  private: void PublishStats();

  /// \brief Callback executed when the task state transition into "ready".
  private: virtual void OnReady();

  /// \brief Callback executed when the task state transition into "running".
  private: virtual void OnRunning();

  /// \brief Callback executed when the task state transition into "finished".
  private: virtual void OnFinished();

  /// \brief Callback executed when a collision is detected for the WAMV.
  private: virtual void OnCollision();

  /// \brief Callback function when collision occurs in the world.
  /// \param[in] _contacts List of all collisions from last simulation iteration
  private: void OnCollisionMsg(ConstContactsPtr &_contacts);

  /// \brief Parse all SDF parameters.
  /// \return True when all parameters were successfully parsed or false
  /// otherwise.
  private: bool ParseSDFParameters();

  /// \brief Parse the joints section of the SDF block.
  /// \return True when all parameters were successfully parsed or false
  /// otherwise.
  private: bool ParseJoints();

  /// \brief A world pointer.
  protected: gazebo::physics::WorldPtr world;

  /// \brief The name of the task.
  protected: std::string taskName = "undefined";

  /// \brief The name of the vehicle to score.
  protected: std::string vehicleName;

  /// \brief Pointer to the vehicle to score.
  protected: gazebo::physics::ModelPtr vehicleModel;

  /// \brief Last collision time.
  protected: gazebo::common::Time lastCollisionTime;

  /// \brief gazebo node pointer
  private: gazebo::transport::NodePtr gzNode;

  /// \brief Collision detection node subscriber
  private: gazebo::transport::SubscriberPtr collisionSub;

  /// \brief gazebo server control publisher
  private: gazebo::transport::PublisherPtr serverControlPub;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Topic where the task stats are published.
  private: std::string taskInfoTopic = "/vrx/task/info";

  /// \brief Bool flag for debug.
  private: bool debug = true;

  /// \brief Topic where debug collision is published.
  private: std::string contactDebugTopic = "/vrx/debug/contact";

  /// \brief The score.
  private: double score = 0.0;

  /// \brief Pointer to the SDF plugin element.
  private: sdf::ElementPtr sdf;

  /// \brief Duration (seconds) of the initial state.
  private: double initialStateDuration = 30.0;

  /// \brief Duration (seconds) of the ready state.
  private: double readyStateDuration = 60.0;

  /// \brief Duration (seconds) of the running state (max task time).
  protected: double runningStateDuration = 300.0;

  /// \brief Absolute time specifying the start of the ready state.
  private: gazebo::common::Time readyTime;

  /// \brief Absolute time specifying the start of the running state.
  private: gazebo::common::Time runningTime;

  /// \brief Absolute time specifying the start of the finish state.
  private: gazebo::common::Time finishTime;

  /// \brief Current time (simulation).
  private: gazebo::common::Time currentTime;

  // \brief Elapsed time since the start of the task (running state).
  private: gazebo::common::Time elapsedTime;

  /// \brief Remaining time since the start of the task (running state).
  private: gazebo::common::Time remainingTime;

  /// \brief Collision buffer.
  private: float CollisionBuffer = 3.0;

  /// \brief Collisions counter.
  private: int collisionCounter = 0;

  /// \brief Collision list.
  private: std::vector<std::string> collisionList;

  /// \brief Collisions timestamps.
  private: std::vector<gazebo::common::Time> collisionTimestamps;

  /// \brief Whether the current task has timed out or not.
  private: bool timedOut = false;

  /// \brief Time at which the last message was sent.
  private: gazebo::common::Time lastStatsSent = gazebo::common::Time::Zero;

  /// \brief The task state.
  private: std::string taskState = "initial";

  /// \brief The next task message to be published.
  protected: vrx_gazebo::Task taskMsg;

  /// \brief ROS Contact Msg.
  private: vrx_gazebo::Contact contactMsg;

  /// \brief The name of the joints to be dettached during ReleaseVehicle().
  private: std::vector<std::string> lockJointNames;

  /// \brief ROS node handle.
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief Publisher for the task state.
  protected: ros::Publisher taskPub;

  /// \brief Publisher for the collision.
  private: ros::Publisher contactPub;

  /// \brief Score in case of timeout - added for Navigation task
  private: double timeoutScore = -1.0;

  /// \brief Whether to shut down after last gate is crossed.
  private: bool perPluginExitOnCompletion = true;
};

#endif
