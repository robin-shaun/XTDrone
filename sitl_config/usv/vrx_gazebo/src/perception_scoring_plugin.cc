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

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/config.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/perception_scoring_plugin.hh"

/* The version of ignition math in Ubuntu Xenial is 2.2.3 and lacks of
 * some features added after that version in the 2.x series */

/* There is a bug in versions for ign-math in Bionic that does not
 * define properly IGNITION_MATH_MAJOR_VERSION. Some magic to check
 * defined variables but empty and assume 3.0.x series */
#define DO_EXPAND(VAL)  VAL ## 1
#define EXPAND(VAL)     DO_EXPAND(VAL)
#if (EXPAND(IGNITION_MATH_MAJOR_VERSION) == 1)
  #define MAJOR_VERSION 3
  #define MINOR_VERSION 0
#else
  #define MAJOR_VERSION IGNITION_MATH_MAJOR_VERSION
  #define MINOR_VERSION IGNITION_MATH_MINOR_VERSION
#endif

#if MAJOR_VERSION == 2 && MINOR_VERSION < 3
  #define ign_math_vector3d_zero ignition::math::Vector3d(0, 0, 0)
#else
  #define ign_math_vector3d_zero ignition::math::Vector3d::Zero
#endif



/////////////////////////////////////////////////
PerceptionObject::PerceptionObject(const double& _time,
               const double& _duration,
               const std::string& _type,
               const std::string& _name,
               const ignition::math::Pose3d& _trialPose,
               const gazebo::physics::WorldPtr _world)
{
  this->time = _time;
  this->duration = _duration;
  this->type = _type;
  this->name = _name;
  this->trialPose = _trialPose;
  #if GAZEBO_MAJOR_VERSION >= 8
    this->modelPtr = _world->EntityByName(this->name);
  #else
    this->modelPtr = _world->GetEntity(this->name);
  #endif
  if (modelPtr)
  {
    #if GAZEBO_MAJOR_VERSION >= 8
      this->origPose = this->modelPtr->WorldPose();
    #else
      this->origPose = this->modelPtr->GetWorldPose().Ign();
    #endif
  }
}

std::string PerceptionObject::Str()
{
  std::string rtn = "\nname: ";
  rtn += this->name;
  rtn += "\ntype: ";
  rtn += this->type;
  rtn += "\ntime: ";
  rtn += std::to_string(this->time);
  rtn +=  "\nduration: ";
  rtn += std::to_string(this->duration);
  rtn +=  "\nerror: ";
  rtn += std::to_string(this->error);
  return rtn;
}

/////////////////////////////////////////////////
void PerceptionObject::SetError(const double& _error)
{
  if (this->active)
    this->error = std::min(2.0, std::min(this->error, _error));
}

/////////////////////////////////////////////////
void PerceptionObject::StartTrial(const gazebo::physics::EntityPtr& _frame)
{
  // Set object pose relative to the specified frame (e.g., the wam-v)
  // Pitch and roll are set to zero as a hack to deal with
  // transients associated with spawning buoys with significant attitude.
  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d framePose(
      _frame->WorldPose().Pos(),
      ignition::math::Quaterniond(0.0, 0.0,
        _frame->WorldPose().Rot().Yaw()));
  #else
    ignition::math::Pose3d framePose(
      _frame->GetWorldPose().pos.Ign(),
      ignition::math::Quaterniond(0.0, 0.0,
        _frame->GetWorldPose().rot.Ign().Yaw()));
  #endif
  ignition::math::Matrix4d transMat(framePose);
  ignition::math::Matrix4d pose_local(this->trialPose);

  this->modelPtr->SetWorldPose((transMat * pose_local).Pose());
  this->modelPtr->SetWorldTwist(ign_math_vector3d_zero,
    ign_math_vector3d_zero);
  this->active = true;
  gzmsg << "PerceptionScoringPlugin: spawning " << this->name << std::endl;
}

/////////////////////////////////////////////////
void PerceptionObject::EndTrial()
{
  this->modelPtr->SetWorldPose(this->origPose);
  this->modelPtr->SetWorldTwist(ign_math_vector3d_zero,
    ign_math_vector3d_zero);
  this->active = false;
  gzmsg << "PerceptionScoringPlugin: despawning " << this->name << std::endl;
}

GZ_REGISTER_WORLD_PLUGIN(PerceptionScoringPlugin)

/////////////////////////////////////////////////
PerceptionScoringPlugin::PerceptionScoringPlugin()
{
  gzmsg << "PerceptionScoringPlugin loaded" << std::endl;
}

/////////////////////////////////////////////////
PerceptionScoringPlugin::~PerceptionScoringPlugin()
{
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::Load(gazebo::physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
  // Base class, also binds the update method for the base class
  ScoringPlugin::Load(_world, _sdf);

  this->world = _world;
  this->sdf = _sdf;

  if (_sdf->HasElement("loop_forever"))
  {
    sdf::ElementPtr loopElem = _sdf->GetElement("loop_forever");
    this->loopForever = loopElem->Get<bool>();
  }

  if (_sdf->HasElement("frame"))
  {
    this->frameName = _sdf->Get<std::string>("frame");
  }

  if (!_sdf->HasElement("object_sequence"))
  {
    gzerr << "PerceptionScoringPlugin: Unable to find <object_sequence> "
      "element\n";
    return;
  }

  sdf::ElementPtr sequence = _sdf->GetElement("object_sequence");

  sdf::ElementPtr objectElem = NULL;
  if (sequence->HasElement("object"))
  {
    objectElem = sequence->GetElement("object");
  }

  while (objectElem)
  {
    // Parse the time.
    if (!objectElem->HasElement("time"))
    {
      gzerr << "PerceptionScoringPlugin: Unable to find <time> in object\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr timeElement = objectElem->GetElement("time");
    double time = timeElement->Get<double>();

    double duration = 5;
    if (objectElem->HasElement("duration"))
    {
      sdf::ElementPtr durationElement = objectElem->GetElement("duration");
      duration = durationElement->Get<double>();
    }

    // Parse the object type.
    if (!objectElem->HasElement("type"))
    {
      gzerr << "PerceptionScoringPlugin: Unable to find <type> in object.\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr typeElement = objectElem->GetElement("type");
    std::string type = typeElement->Get<std::string>();

    // Parse the object name - this is what must be matched id success.
    if (!objectElem->HasElement("name"))
    {
      gzerr << "PerceptionScoringPlugin: Unable to find <name> in object.\n";
      objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr nameElement = objectElem->GetElement("name");
    std::string name = nameElement->Get<std::string>();

    // Parse the object pose
    if (!objectElem->HasElement("pose"))
    {
      gzerr << "PerceptionScoringPlugin: Unable to find <pose> in object.\n";
        objectElem = objectElem->GetNextElement("object");
      continue;
    }
    sdf::ElementPtr poseElement = objectElem->GetElement("pose");
    ignition::math::Pose3d pose = poseElement->Get<ignition::math::Pose3d>();

    // Add the object to the collection.
    PerceptionObject obj(time, duration, type, name, pose, _world);
    this->objects.push_back(obj);

    objectElem = objectElem->GetNextElement("object");
  }

  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastUpdateTime = this->world->SimTime();
  #else
    this->lastUpdateTime = this->world->GetSimTime();
  #endif

  // Optional: ROS namespace.
  if (_sdf->HasElement("robot_namespace"))
    this->ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  // Optional: ROS topic.
  this->objectTopic = "/vrx/perception/landmark";
  if (_sdf->HasElement("landmark_topic"))
  {
    this->objectTopic = _sdf->GetElement("landmark_topic")->Get<std::string>();
  }

  this->Restart();

  this->connection = gazebo::event::Events::ConnectWorldUpdateEnd(
      boost::bind(&PerceptionScoringPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::Restart()
{
  for (auto& obj : this->objects)
  {
    // reset all objects' errors
    obj.error = 10.0;
    // bump all objs time to start again
    #if GAZEBO_MAJOR_VERSION >= 8
      obj.time += this->world->SimTime().Double();
    #else
      obj.time += this->world->GetSimTime().Double();
    #endif
  }
  gzmsg << "Object population restarted" << std::endl;
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::OnUpdate()
{
  // if have not finished the load, skip
  if (!this->frameName.empty())
  {
    // get frame of robot
    #if GAZEBO_MAJOR_VERSION >= 8
      this->frame =
        this->world->EntityByName(this->frameName);
    #else
      this->frame =
        this->world->GetEntity(this->frameName);
    #endif
    // make sure it is a real frame
    if (!this->frame)
    {
      gzwarn << std::string("The frame '") << this->frameName
        << "' does not exist" << std::endl;
      return;
    }
    if (!this->frame->HasType(gazebo::physics::Base::LINK) &&
        !this->frame->HasType(gazebo::physics::Base::MODEL))
    {
      gzwarn << "'frame' tag must list the name of a link or model"
        << std::endl;
      return;
    }
  }
  // look at all objects
  for (auto& obj : this->objects)
  {
    // if time to spawn an object
    if (this->ElapsedTime() > obj.time &&
        this->ElapsedTime() < obj.time + obj.duration &&
        !obj.active)
    {
      // increment the atempt balance for this new obj
      this->attemptBal += 1;
      obj.StartTrial(this->frame);
      ROS_INFO_NAMED("PerceptionScoring",
        "New Attempt Balance: %d", this->attemptBal);
    }
    // if time to despawn and object
    if (this->ElapsedTime() > obj.time + obj.duration &&
        obj.active)
    {
      // prevent negative attemp balance
      if (this->attemptBal > 0)
        this->attemptBal -= 1;
      // inc objects despawned
      this->objectsDespawned += 1;
      obj.EndTrial();

      // Add the score for this object.
      this->SetScore(this->Score() + obj.error);

      ROS_INFO_NAMED("PerceptionScoring",
        "New Attempt Balance: %d", this->attemptBal);
    }
  }
  // if we have finished
  if (this->objectsDespawned == this->objects.size() &&
      this->TaskState() != "finished")
  {
    // publish string summarizing the objects
    for (auto& obj : this->objects)
    {
      ROS_INFO_NAMED("PerceptionScoring", "%s", obj.Str().c_str());
    }

    // Run score is the mean error per object.
    this->SetScore(this->Score() / this->objects.size());
    ROS_INFO_NAMED("Perception run score: ", "%f", this->Score());

    // if loop, restart
    if (this->loopForever)
    {
      this->objectsDespawned = 0;
      this->Restart();
    }
    else
    {
      this->Finish();
    }
  }
}

/////////////////////////////////////////////////
void PerceptionScoringPlugin::OnAttempt(
  const geographic_msgs::GeoPoseStamped::ConstPtr &_msg)
{
  // only accept an attempt if there are any in the attempt balance
  if (this->attemptBal == 0)
  {
    ROS_WARN_NAMED("PerceptionScoring",
      "Attempt Balance is 0, no attempts currently allowed. Ignoring.");
    return;
  }
  else
  {
    // burn one attempt
    this->attemptBal -= 1;
    ROS_INFO_NAMED("PerceptionScoring",
      "New Attempt Balance: %d", this->attemptBal);
  }
  for (auto& obj : this->objects)
  {
    // if attempt correct type
    if (obj.type == _msg->header.frame_id)
    {
      // Convert geo pose to Gazebo pose
      // Note - this is used in a few different VRX plugins, may want to have a
      // separate library?
      // Convert lat/lon to local
      // Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
      ignition::math::Vector3d scVec(_msg->pose.position.latitude,
        _msg->pose.position.longitude, 0);
      #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d cartVec =
        this->world->SphericalCoords()->LocalFromSpherical(scVec);
      #else
        ignition::math::Vector3d cartVec =
        this->world->GetSphericalCoordinates()->LocalFromSpherical(scVec);
      #endif
      // Get current pose of the current object
      #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d truePose = obj.modelPtr->WorldPose();
      #else
        ignition::math::Pose3d truePose = obj.modelPtr->GetWorldPose().Ign();
      #endif
      // 2D Error
      obj.SetError(sqrt(pow(cartVec.X() - truePose.Pos().X(), 2)+
        pow(cartVec.Y() - truePose.Pos().Y(), 2)));
    }
  }
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  // Subscribe
  this->nh = ros::NodeHandle(this->ns);
  this->objectSub = this->nh.subscribe(this->objectTopic, 1,
    &PerceptionScoringPlugin::OnAttempt, this);
}

//////////////////////////////////////////////////
void PerceptionScoringPlugin::ReleaseVehicle()
{
  // Avoid releasing the vehicle by overriding this function.
  return;
}
