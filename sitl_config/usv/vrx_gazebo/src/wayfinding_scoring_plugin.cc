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

#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <cmath>
#include <gazebo/common/Console.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Model.hh>

#include "vrx_gazebo/waypoint_markers.hh"
#include "vrx_gazebo/wayfinding_scoring_plugin.hh"

/////////////////////////////////////////////////
WayfindingScoringPlugin::WayfindingScoringPlugin()
  : waypointMarkers("waypoint_marker")
{
  gzmsg << "Wayfinding scoring plugin loaded" << std::endl;
  this->timer.Stop();
  this->timer.Reset();
}

/////////////////////////////////////////////////
void WayfindingScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);
  this->sdf = _sdf;
  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  // A waypoints element is required.
  if (!this->sdf->HasElement("waypoints"))
  {
    gzerr << "Unable to find <waypoints> element in SDF." << std::endl;
    return;
  }
  auto waypointsElem = this->sdf->GetElement("waypoints");

  // We need at least one waypoint
  if (!waypointsElem->HasElement("waypoint"))
  {
    gzerr << "Unable to find <waypoint> element in SDF." << std::endl;
    return;
  }
  auto waypointElem = waypointsElem->GetElement("waypoint");

  while (waypointElem)
  {
    ignition::math::Vector3d latlonyaw =
      waypointElem->Get<ignition::math::Vector3d>("pose");

    // Convert lat/lon to local
    //  snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
    ignition::math::Vector3d scVec(latlonyaw.X(), latlonyaw.Y(), 0.0);

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d cartVec =
      _world->SphericalCoords()->LocalFromSpherical(scVec);
#else
    ignition::math::Vector3d cartVec =
      _world->GetSphericalCoordinates()->LocalFromSpherical(scVec);
#endif

    cartVec.Z() = latlonyaw.Z();

    // Set up relevant vectors
    this->sphericalWaypoints.push_back(latlonyaw);
    this->localWaypoints.push_back(cartVec);

    // Print some debugging messages
    gzmsg << "Waypoint, Spherical: Lat = " << latlonyaw.X()
          << " Lon = " << latlonyaw.Y() << std::endl;
    gzmsg << "Waypoint, Local: X = " << cartVec.X()
          << " Y = " << cartVec.Y() << " Yaw = " << cartVec.Z() << std::endl;

    waypointElem = waypointElem->GetNextElement("waypoint");
  }

  // Setup ROS node and publisher
  this->rosNode.reset(new ros::NodeHandle());
  if (_sdf->HasElement("waypoints_topic"))
  {
    this->waypointsTopic = _sdf->Get<std::string>("waypoints_topic");
  }
  this->waypointsPub =
    this->rosNode->advertise<geographic_msgs::GeoPath>(
      this->waypointsTopic, 10, true);

  if (_sdf->HasElement("min_errors_topic"))
  {
    this->minErrorsTopic = _sdf->Get<std::string>("min_errors_topic");
  }
  this->minErrorsPub =
    this->rosNode->advertise<std_msgs::Float64MultiArray>(
      this->minErrorsTopic, 100);

  if (_sdf->HasElement("mean_error_topic"))
  {
    this->meanErrorTopic = _sdf->Get<std::string>("mean_error_topic");
  }
  this->meanErrorPub =
    this->rosNode->advertise<std_msgs::Float64>(
      this->meanErrorTopic, 100);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&WayfindingScoringPlugin::Update, this));

  // Publish waypoint markers
  if (_sdf->HasElement("markers"))
  {
    this->waypointMarkers.Load(_sdf->GetElement("markers"));
    if (this->waypointMarkers.IsAvailable())
    {
      int markerId = 0;
      for (const auto waypoint : this->localWaypoints)
      {
        if (!this->waypointMarkers.DrawMarker(markerId, waypoint.X(),
            waypoint.Y(), waypoint.Z(), std::to_string(markerId)))
        {
          gzerr << "Error creating visual marker" << std::endl;
        }
        markerId++;
      }
    }
    else
    {
      gzwarn << "Cannot display gazebo markers (Gazebo version < 8)"
             << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->vehicleModel)
  {
    #if GAZEBO_MAJOR_VERSION >= 8
      this->vehicleModel = this->world->ModelByName(this->vehicleName);
    #else
      this->vehicleModel = this->world->GetModel(this->vehicleName);
    #endif
    if (!this->vehicleModel)
      return;
  }

  // Nothing to do if the task is not in "running" state.
  if (this->ScoringPlugin::TaskState() != "running")
    return;

  std_msgs::Float64MultiArray minErrorsMsg;
  std_msgs::Float64 meanErrorMsg;

  #if GAZEBO_MAJOR_VERSION >= 8
    const auto robotPose = this->vehicleModel->WorldPose();
  #else
    const auto robotPose = this->vehicleModel->GetWorldPose().Ign();
  #endif
  double currentHeading = robotPose.Rot().Euler().Z();

  double currentTotalError = 0;

  for (unsigned i = 0; i < this->localWaypoints.size(); ++i)
  {
    const ignition::math::Vector3d wp = this->localWaypoints[i];
    double dx   =  wp.X() - robotPose.Pos().X();
    double dy   =  wp.Y() - robotPose.Pos().Y();
    double dhdg =  abs(wp.Z() - currentHeading);
    double headError = 1 - abs(dhdg - M_PI)/M_PI;

    double poseError =  sqrt(pow(dx, 2) + pow(dy, 2)) + headError;

    // If this is the first time through, minError == poseError
    if (i == this->minErrors.size())
    {
      this->minErrors.push_back(poseError);
    }

    // If poseError is smaller than the minimum, update the minimum
    if (poseError < this->minErrors.at(i))
    {
      this->minErrors.at(i) = poseError;
    }

    // add current minimum to current total error
    currentTotalError += this->minErrors.at(i);
  }

  this->meanError = currentTotalError / this->localWaypoints.size();

  // Set up multi array dimensions
  minErrorsMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  minErrorsMsg.layout.dim[0].label = "minimum errors";
  minErrorsMsg.layout.dim[0].size = this->localWaypoints.size();
  minErrorsMsg.layout.dim[0].stride = this->localWaypoints.size();

  minErrorsMsg.data = this->minErrors;
  meanErrorMsg.data = this->meanError;

  // Publish at 1 Hz.
  if (this->timer.GetElapsed() >= gazebo::common::Time(1.0))
  {
    this->minErrorsPub.publish(minErrorsMsg);
    this->meanErrorPub.publish(meanErrorMsg);
    this->timer.Reset();
    this->timer.Start();
  }

  this->ScoringPlugin::SetScore(this->meanError);
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::PublishWaypoints()
{
  gzmsg << "Publishing Waypoints" << std::endl;
  geographic_msgs::GeoPoseStamped wp_msg;
  geographic_msgs::GeoPath path_msg;

  path_msg.header.stamp = ros::Time::now();

  for (auto wp : this->sphericalWaypoints)
  {
    wp_msg.pose.position.latitude  = wp.X();
    wp_msg.pose.position.longitude = wp.Y();
    wp_msg.pose.position.altitude  = 0.0;

    const ignition::math::Quaternion<double> orientation(0.0, 0.0, wp.Z());

    wp_msg.pose.orientation.x = orientation.X();
    wp_msg.pose.orientation.y = orientation.Y();
    wp_msg.pose.orientation.z = orientation.Z();
    wp_msg.pose.orientation.w = orientation.W();

    wp_msg.header.stamp = ros::Time::now();
    path_msg.poses.push_back(wp_msg);
  }
  this->waypointsPub.publish(path_msg);
}

//////////////////////////////////////////////////
void WayfindingScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;
  this->PublishWaypoints();
}


//////////////////////////////////////////////////
void WayfindingScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
  this->timer.Start();
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(WayfindingScoringPlugin)
