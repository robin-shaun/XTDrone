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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include "vrx_gazebo/scoring_plugin.hh"

/////////////////////////////////////////////////
ScoringPlugin::ScoringPlugin()
    : WorldPlugin(), gzNode(new gazebo::transport::Node()) {
}

void ScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "ScoringPlugin::Load(): NULL world pointer");
  GZ_ASSERT(_sdf,   "ScoringPlugin::Load(): NULL _sdf pointer");

  this->world = _world;
  this->sdf = _sdf;

  // SDF.
  if (!this->ParseSDFParameters())
  {
    gzerr << "Scoring disabled" << std::endl;
    return;
  }

  this->readyTime.Set(this->initialStateDuration);
  this->runningTime = this->readyTime + this->readyStateDuration;
  this->finishTime = this->runningTime + this->runningStateDuration;

  // Prepopulate the task msg.
  this->taskMsg.name = this->taskName;
  this->taskMsg.ready_time.fromSec(this->readyTime.Double());
  this->taskMsg.running_time.fromSec(this->runningTime.Double());
  this->UpdateTaskMessage();

  // Initialize ROS transport.
  this->rosNode.reset(new ros::NodeHandle());
  this->taskPub = this->rosNode->advertise<vrx_gazebo::Task>
    (this->taskInfoTopic, 100);
  this->contactPub = this->rosNode->advertise<vrx_gazebo::Contact>
    (this->contactDebugTopic, 100);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScoringPlugin::Update, this));

  gzNode->Init();
#if GAZEBO_MAJOR_VERSION >= 8
  std::string worldName = this->world->Name();
#else
  std::string worldName = this->world->GetName();
#endif
  std::string collisionTopic =
    std::string("/gazebo/") + worldName + std::string("/physics/contacts");
  collisionSub = gzNode->Subscribe(collisionTopic,
                                          &ScoringPlugin::OnCollisionMsg, this);

  if (char* env_dbg = std::getenv("VRX_DEBUG"))
  {
    if (std::string(env_dbg) == "false")
      this->debug = false;
  }
  else
  {
    gzwarn << "VRX_DEBUG enviornment variable not set, defaulting to true"
      << std::endl;
  }
  this->serverControlPub =
    this->gzNode->Advertise<gazebo::msgs::ServerControl>
    ("/gazebo/server/control");
}

//////////////////////////////////////////////////
double ScoringPlugin::Score() const
{
  return this->score;
}

//////////////////////////////////////////////////
void ScoringPlugin::SetScore(double _newScore)
{
  if (this->TaskState() == "running")
    this->score = _newScore;
}

//////////////////////////////////////////////////
std::string ScoringPlugin::TaskName() const
{
  return this->taskName;
}

//////////////////////////////////////////////////
std::string ScoringPlugin::TaskState() const
{
  return this->taskState;
}

//////////////////////////////////////////////////
gazebo::common::Time ScoringPlugin::ElapsedTime() const
{
  return this->elapsedTime;
}

//////////////////////////////////////////////////
gazebo::common::Time ScoringPlugin::RemainingTime() const
{
  return this->remainingTime;
}

//////////////////////////////////////////////////
void ScoringPlugin::Finish()
{
  if (this->taskState == "finished")
    return;

  this->taskState = "finished";
  this->OnFinished();
}

//////////////////////////////////////////////////
void ScoringPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  #if GAZEBO_MAJOR_VERSION >= 8
    if (!this->vehicleModel)
      this->vehicleModel = this->world->ModelByName(this->vehicleName);
  #else
    if (!this->vehicleModel)
      this->vehicleModel = this->world->GetModel(this->vehicleName);
  #endif

  this->UpdateTime();
  this->UpdateTaskState();
  this->PublishStats();
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTime()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    this->currentTime = this->world->SimTime();
  #else
    this->currentTime = this->world->GetSimTime();
  #endif

  if (this->taskState == "running")
  {
    this->elapsedTime = this->currentTime - this->runningTime;
    this->remainingTime = this->finishTime - this->currentTime;
    this->timedOut = this->remainingTime <= gazebo::common::Time::Zero;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTaskState()
{
  if (this->taskState == "initial" &&
      this->currentTime >= this->readyTime)
  {
    this->taskState = "ready";
    this->ReleaseVehicle();
    this->OnReady();
    return;
  }

  if (this->taskState == "ready" &&
      this->currentTime >= this->runningTime)
  {
    this->taskState = "running";
    this->OnRunning();
    return;
  }

  if (this->taskState == "running" && this->timedOut)
  {
    this->taskState = "finished";
    this->OnFinished();
    return;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::UpdateTaskMessage()
{
  this->taskMsg.state = this->taskState;
  this->taskMsg.elapsed_time.fromSec(this->elapsedTime.Double());
  this->taskMsg.remaining_time.fromSec(this->remainingTime.Double());
  this->taskMsg.timed_out = this->timedOut;
  this->taskMsg.score = this->score;
}

//////////////////////////////////////////////////
void ScoringPlugin::PublishStats()
{
  this->UpdateTaskMessage();

  // We publish stats at 1Hz.
  if (this->currentTime - this->lastStatsSent >= gazebo::common::Time(1, 0))
  {
    this->taskPub.publish(this->taskMsg);
    this->lastStatsSent = this->currentTime;
  }
}

//////////////////////////////////////////////////
void ScoringPlugin::ReleaseVehicle()
{
  if (!this->vehicleModel || this->lockJointNames.empty())
    return;

  for (auto jointName : this->lockJointNames)
  {
    auto joint = this->vehicleModel->GetJoint(jointName);
    if (joint)
      joint->Detach();
    else
      gzerr << "Unable to release [" << jointName << "]" << std::endl;
  }

  this->lockJointNames.clear();

  gzmsg << "Vehicle released" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
}

//////////////////////////////////////////////////
void ScoringPlugin::OnFinished()
{
  gzmsg << ros::Time::now() << "  OnFinished" << std::endl;
  // If a timeoutScore was specified, use it.
  if (this->timedOut && this->timeoutScore > 0.0)
  {
    this->score = this->timeoutScore;
  }
  this->UpdateTaskMessage();
  this->taskPub.publish(this->taskMsg);
  this->Exit();
}

//////////////////////////////////////////////////
void ScoringPlugin::OnCollision()
{
}

//////////////////////////////////////////////////
void ScoringPlugin::OnCollisionMsg(ConstContactsPtr &_contacts) {
  // loop though collisions, if any include the wamv, increment collision
  // counter
  for (unsigned int i = 0; i < _contacts->contact_size(); ++i) {
    std::string wamvCollisionStr1 = _contacts->contact(i).collision1();
    std::string wamvCollisionStr2 = _contacts->contact(i).collision2();
    std::string wamvCollisionSubStr1 =
        wamvCollisionStr1.substr(0, wamvCollisionStr1.find("lump"));
    std::string wamvCollisionSubStr2 =
        wamvCollisionStr2.substr(0, wamvCollisionStr2.find("lump"));

    bool isWamvHit =
        wamvCollisionSubStr1 == "wamv::base_link::base_link_fixed_joint_" ||
        wamvCollisionSubStr2 == "wamv::base_link::base_link_fixed_joint_";
    bool isHitBufferPassed = this->currentTime - this->lastCollisionTime >
                             gazebo::common::Time(CollisionBuffer, 0);

    // publish a Contact MSG
    if (isWamvHit && this->debug) {
      this->contactMsg.header.stamp = ros::Time::now();
      this->contactMsg.collision1 = _contacts->contact(i).collision1();
      this->contactMsg.collision2 = _contacts->contact(i).collision2();
      this->contactPub.publish(this->contactMsg);
    }

    if (isWamvHit && isHitBufferPassed) {
      this->collisionCounter++;
      gzmsg << "[" << this->collisionCounter
            << "] New collision counted between ["
            << _contacts->contact(i).collision1() << "] and ["
            << _contacts->contact(i).collision2() << "]" << std::endl;
      // Uncomment to get details of collisions
      // gzdbg << _contacts->contact(i).DebugString() << std::endl;
#if GAZEBO_MAJOR_VERSION >= 8
      this->lastCollisionTime = this->world->SimTime();
#else
      this->lastCollisionTime = this->world->GetSimTime();
#endif
      this->collisionList.push_back(
          _contacts->contact(i).collision1() +
          std::string(" || ") + _contacts->contact(i).collision2());
      this->collisionTimestamps.push_back(this->currentTime);
      this->OnCollision();
      return;
    }
  }
}

//////////////////////////////////////////////////
bool ScoringPlugin::ParseSDFParameters()
{
  // This is a required element.
  if (!this->sdf->HasElement("vehicle"))
  {
    gzerr << "Unable to find <vehicle> element in SDF." << std::endl;
    return false;
  }
  this->vehicleName = this->sdf->Get<std::string>("vehicle");

  // This is a required element.
  if (!this->sdf->HasElement("task_name"))
  {
    gzerr << "Unable to find <task_name> element in SDF." << std::endl;
    return false;
  }
  this->taskName = this->sdf->Get<std::string>("task_name");

  // This is an optional element.
  if (this->sdf->HasElement("task_info_topic"))
    this->taskInfoTopic = this->sdf->Get<std::string>("task_info_topic");

  // This is an optional element.
  if (this->sdf->HasElement("contact_debug_topic"))
    this->contactDebugTopic = this->sdf->Get<std::string>
      ("contact_debug_topic");

  // This is an optional element.
  if (this->sdf->HasElement("per_plugin_exit_on_completion"))
    this->perPluginExitOnCompletion = this->sdf->Get<bool>(
      "per_plugin_exit_on_completion");

  // This is an optional element.
  if (this->sdf->HasElement("initial_state_duration"))
  {
    double value = this->sdf->Get<double>("initial_state_duration");
    if (value < 0)
    {
      gzerr << "<initial_state_duration> value should not be negative."
            << std::endl;
      return false;
    }
    this->initialStateDuration = value;
  }

  // This is an optional element.
  if (this->sdf->HasElement("ready_state_duration"))
  {
    double value = this->sdf->Get<double>("ready_state_duration");
    if (value < 0)
    {
      gzerr << "<ready_state_duration> value should not be negative."
            << std::endl;
      return false;
    }

    this->readyStateDuration = value;
  }

  // This is an optional element.
  if (this->sdf->HasElement("running_state_duration"))
  {
    double value = this->sdf->Get<double>("running_state_duration");
    if (value < 0)
    {
      gzerr << "<running_state_duration> value should not be negative."
            << std::endl;
      return false;
    }
    this->runningStateDuration = value;
  }

  // This is an optional element.
  if (this->sdf->HasElement("collision_buffer"))
  {
    this->CollisionBuffer = this->sdf->Get<float>("collision_buffer");
  }

  return this->ParseJoints();
}

//////////////////////////////////////////////////
bool ScoringPlugin::ParseJoints()
{
  // Optional element.
  if (this->sdf->HasElement("release_joints"))
  {
    auto releaseJointsElem = this->sdf->GetElement("release_joints");

    // We need at least one joint.
    if (!releaseJointsElem->HasElement("joint"))
    {
      gzerr << "Unable to find <joint> element in SDF." << std::endl;
      return false;
    }

    auto jointElem = releaseJointsElem->GetElement("joint");

    // Parse a new joint to be released.
    while (jointElem)
    {
      // The joint's name.
      if (!jointElem->HasElement("name"))
      {
        gzerr << "Unable to find <name> element in SDF." << std::endl;
        return false;
      }

      const std::string jointName = jointElem->Get<std::string>("name");
      this->lockJointNames.push_back(jointName);

      // Parse the next joint.
      jointElem = jointElem->GetNextElement("joint");
    }
  }

  return true;
}

void ScoringPlugin::Exit()
{
  bool exit = this->perPluginExitOnCompletion;

  char* env = std::getenv("VRX_EXIT_ON_COMPLETION");
  if (env != nullptr && std::string(env) == "true")
  {
    // Overwrite class variable if environment variable is specified
    exit = true;
  }

  if (exit)
  {
    // shutdown gazebo
    gazebo::msgs::ServerControl msg;
    msg.set_stop(true);
    this->serverControlPub->Publish(msg);
    // shutdown gazebo
    if (ros::ok())
      ros::shutdown();
  }
  else
  {
    gzerr << "VRX_EXIT_ON_COMPLETION and <per_plugin_exit_on_completion> "
      << "both not set, will not shutdown on ScoringPlugin::Exit()"
      << std::endl;
    ROS_ERROR_STREAM(
      "VRX_EXIT_ON_COMPLETION and <per_plugin_exit_on_completion> "
      << "both not set, will not shutdown on ScoringPlugin::Exit()");
  }
  return;
}

void ScoringPlugin::SetTimeoutScore(double _timeoutScore)
{
  this->timeoutScore = _timeoutScore;
}

double ScoringPlugin::GetTimeoutScore()
{
  return this->timeoutScore;
}

double ScoringPlugin::GetRunningStateDuration()
{
  return this->runningStateDuration;
}
