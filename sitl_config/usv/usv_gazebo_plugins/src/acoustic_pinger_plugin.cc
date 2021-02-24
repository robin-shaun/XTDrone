/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 *  \Original plugin from gazebo_plugins package 
 *  \(https://github.com/ros-simulation/gazebo_ros_pkgs), 
 *  \modified to implement a simulated pinger sensor for Maritime ASVs
 *  \Modifications by Jonathan Wheare.
 *  
 */

#include <usv_msgs/RangeBearing.h>
#include <cmath>
#include <ignition/math/Pose3.hh>
#include "usv_gazebo_plugins/acoustic_pinger_plugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AcousticPinger);

//////////////////////////////////////////////////
AcousticPinger::AcousticPinger()
{
}

//////////////////////////////////////////////////
AcousticPinger::~AcousticPinger()
{
}

//////////////////////////////////////////////////
void AcousticPinger::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store pointer to model for later use.
  this->model = _parent;

  // From gazebo_ros_color plugin.
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Make sure the ROS node for Gazebo has already been initialised.
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("usv_gazebo_acoustic_pinger_plugin", "A ROS node for"
      " Gazebo hasn't been initialised, unable to load plugin. Load the Gazebo "
      "system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // From gazebo_ros_color plugin
  // Load namespace from SDF if available. Otherwise, use the model name.
  std::string modelName = _parent->GetName();
  auto delim = modelName.find(":");
  if (delim != std::string::npos)
    modelName = modelName.substr(0, delim);

  // Initialise the namespace.
  std::string ns = modelName;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
  {
    ROS_DEBUG_NAMED("usv_gazebo_acoustic_pinger_plugin",
      "missing <robotNamespace>, defaulting to %s", ns.c_str());
  }

  // Set the frame_id. Defaults to "pinger".
  this->frameId = "pinger";
  if (_sdf->HasElement("frameId"))
    this->frameId = _sdf->GetElement("frameId")->Get<std::string>();

  // Load topic from SDF if available.
  std::string topicName = "/pinger/range_bearing";
  if (_sdf->HasElement("topicName"))
    topicName = _sdf->GetElement("topicName")->Get<std::string>();
  else
  {
    ROS_INFO_NAMED("usv_gazebo_acoustic_pinger_plugin",
      "missing <topicName>, defaulting to %s", topicName.c_str());
  }

  // Set the topic to be used to publish the sensor message.
  std::string setPositionTopicName = "/pinger/set_pinger_position";
  if (_sdf->HasElement("setPositionTopicName"))
  {
    setPositionTopicName =
      _sdf->GetElement("setPositionTopicName")->Get<std::string>();
  }
  else
  {
    ROS_INFO_NAMED("usv_gazebo_acoustic_pinger_plugin",
      "missing <setPositionTopicName>, defaulting to %s", topicName.c_str());
  }

  // Initialise pinger position. Defaults to origin.
  this->position = ignition::math::Vector3d::Zero;
  if (_sdf->HasElement("position"))
    this->position = _sdf->Get<ignition::math::Vector3d>("position");

  // Initialise update rate. Default to 1 reading per second.
  this->updateRate = 1.0f;
  if (_sdf->HasElement("updateRate"))
    this->updateRate = _sdf->Get<float>("updateRate");

  // From Brian Bingham's rangebearing_gazebo_plugin.
  // Noise setup and parse SDF.
  if (_sdf->HasElement("rangeNoise"))
  {
    sdf::ElementPtr rangeNoiseElem = _sdf->GetElement("rangeNoise");
    // Note - it is hardcoded into the NoiseFactory.cc that the SDF
    // element be "noise".
    if (rangeNoiseElem->HasElement("noise"))
    {
      this->rangeNoise = sensors::NoiseFactory::NewNoiseModel(
        rangeNoiseElem->GetElement("noise"));
    }
    else
    {
      ROS_WARN("usv_gazebo_acoustic_pinger_plugin: "
               "The rangeNoise SDF element must contain noise tag");
    }
  }
  else
  {
    ROS_INFO("usv_gazebo_acoustic_pinger_plugin: "
             "No rangeNoise tag found, no noise added to measurements");
  }

  // Load the noise model from the SDF file.
  if (_sdf->HasElement("bearingNoise"))
  {
    sdf::ElementPtr bearingNoiseElem = _sdf->GetElement("bearingNoise");
    // Note - it is hardcoded into the NoiseFactory.cc that the SDF
    // element be "noise".
    if (bearingNoiseElem->HasElement("noise"))
    {
      this->bearingNoise = sensors::NoiseFactory::NewNoiseModel(
        bearingNoiseElem->GetElement("noise"));
    }
    else
    {
      ROS_WARN("usv_gazebo_acoustic_pinger_plugin: "
               "The bearingNoise SDF element must contain noise tag");
    }
  }
  else
  {
    ROS_INFO("usv_gazebo_acoustic_pinger_plugin: "
             "No bearingNoise tag found, no noise added to measurements");
  }

  if (_sdf->HasElement("elevationNoise"))
  {
    sdf::ElementPtr elevationNoiseElem = _sdf->GetElement("elevationNoise");
    // Note - it is hardcoded into the NoiseFactory.cc that the SDF
    // element be "noise".
    if (elevationNoiseElem->HasElement("noise"))
    {
      this->elevationNoise = sensors::NoiseFactory::NewNoiseModel(
        elevationNoiseElem->GetElement("noise"));
    }
    else
    {
      ROS_WARN("usv_gazebo_acoustic_pinger_plugin: "
               "The elevationNoise SDF element must contain noise tag");
    }
  }
  else
  {
    ROS_INFO("usv_gazebo_acoustic_pinger_plugin: "
             "No elevationNoise tag found, no noise added to measurements");
  }

  // initialise the ros handle.
  this->rosNodeHandle.reset(new ros::NodeHandle(ns));

  // setup the publisher.
  this->rangeBearingPub =
    this->rosNodeHandle->advertise<usv_msgs::RangeBearing>(
      std::string(topicName), 1);

  this->setPositionSub = this->rosNodeHandle->subscribe(
    setPositionTopicName, 1, &AcousticPinger::PingerPositionCallback, this);

  // Initialise the time with world time.
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastUpdateTime = this->model->GetWorld()->SimTime();
#else
  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
#endif

  // connect the update function to the world update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AcousticPinger::Update, this));
}

//////////////////////////////////////////////////
void AcousticPinger::Update()
{
  // Test to see if it's time to generate a sensor reading.
#if GAZEBO_MAJOR_VERSION >= 8
  if ((this->model->GetWorld()->SimTime() - this->lastUpdateTime) >
      (1.0f / this->updateRate))
#else
  if ((this->model->GetWorld()->GetSimTime() - this->lastUpdateTime) >
      (1.0f / this->updateRate))
#endif
  {
    // lock the thread to protect this->position vector.
    std::lock_guard<std::mutex> lock(this->mutex);

#if GAZEBO_MAJOR_VERSION >= 8
    this->lastUpdateTime = this->model->GetWorld()->SimTime();
    // Find the pose of the model.
    ignition::math::Pose3d modelPose = this->model->WorldPose();
#else
    this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
    // Find the pose of the model.
    ignition::math::Pose3d modelPose = this->model->GetWorldPose().Ign();
#endif

    // Direction vector to the pinger from the USV.
    ignition::math::Vector3d direction = this->position - modelPose.Pos();

    // Sensor reading is in the sensor frame. Rotate the direction vector into
    // the frame of reference of the sensor.
    ignition::math::Vector3d directionSensorFrame =
      modelPose.Rot().RotateVectorReverse(direction);

    // Generate a 2d vector for elevation calculation.
    ignition::math::Vector3d directionSensorFrame2d =
      ignition::math::Vector3d(
        directionSensorFrame.X(), directionSensorFrame.Y(), 0);

    // bearing is calculated by finding the world frame direction vector
    // and transforming into the pose of the sensor.  Bearing is calculated
    // using the atan2 function of the x and y components of the transformed
    // vector.  The elevation is calculated from the length of the 2D only
    // and the z component of the sensor frame vector.
    double bearing = atan2(directionSensorFrame.Y(), directionSensorFrame.X());
    double range = directionSensorFrame.Length();
    double elevation =
      atan2(directionSensorFrame.Z(), directionSensorFrame2d.Length());

    // Apply noise to each measurement.
    // From Brian Binghams rangebearing_gazebo_plugin.
    if (this->rangeNoise != nullptr)
      range = this->rangeNoise->Apply(range);
    if (this->bearingNoise != nullptr)
      bearing = this->bearingNoise->Apply(bearing);
    if (this->elevationNoise != nullptr)
      elevation  = this->elevationNoise->Apply(elevation);

    // Publish a ROS message.
    usv_msgs::RangeBearing msg;
    // generate ROS header. Sequence number is automatically populated.
    msg.header.stamp = ros::Time(this->lastUpdateTime.sec,
      this->lastUpdateTime.nsec);
    // frame_id is neccesary for finding the tf transform.  The frame_id is
    // specified in the sdf file.
    msg.header.frame_id = this->frameId;
    // Fill out the members of the message.
    msg.range = range;
    msg.bearing = bearing;
    msg.elevation = elevation;

    // publish range and bearing message.
    this->rangeBearingPub.publish(msg);
  }
}

//////////////////////////////////////////////////
void AcousticPinger::PingerPositionCallback(
  const geometry_msgs::Vector3ConstPtr &_pos)
{
  // Mutex added to prevent simulataneous reads and writes of mutex.
  std::lock_guard<std::mutex> lock(this->mutex);
  this->position = ignition::math::Vector3d(_pos->x, _pos->y, _pos->z);
}
