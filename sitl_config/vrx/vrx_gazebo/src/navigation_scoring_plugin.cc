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

#include "vrx_gazebo/navigation_scoring_plugin.hh"
#include <cmath>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/physics/Link.hh>

/////////////////////////////////////////////////
NavigationScoringPlugin::Gate::Gate(
    const gazebo::physics::LinkPtr _leftMarkerModel,
    const gazebo::physics::LinkPtr _rightMarkerModel)
  : leftMarkerModel(_leftMarkerModel),
    rightMarkerModel(_rightMarkerModel)
{
  this->Update();
}

/////////////////////////////////////////////////
void NavigationScoringPlugin::Gate::Update()
{
  if (!this->leftMarkerModel || !this->rightMarkerModel)
    return;

  // The pose of the markers delimiting the gate.
#if GAZEBO_MAJOR_VERSION >= 8
  const auto leftMarkerPose = this->leftMarkerModel->WorldPose();
  const auto rightMarkerPose = this->rightMarkerModel->WorldPose();
#else
  const auto leftMarkerPose = this->leftMarkerModel->GetWorldPose().Ign();
  const auto rightMarkerPose = this->rightMarkerModel->GetWorldPose().Ign();
#endif

  // Unit vector from the left marker to the right one.
  auto v1 = leftMarkerPose.Pos() - rightMarkerPose.Pos();
  v1.Normalize();

  // Unit vector perpendicular to v1 in the direction we like to cross gates.
  const auto v2 = ignition::math::Vector3d::UnitZ.Cross(v1);

  // This is the center point of the gate.
  const auto middle = (leftMarkerPose.Pos() + rightMarkerPose.Pos()) / 2.0;

  // Yaw of the gate in world coordinates.
  const auto yaw = atan2(v2.Y(), v2.X());

  // The updated pose.
  this->pose.Set(middle, ignition::math::Vector3d(0, 0, yaw));

  // The updated width.
  this->width = leftMarkerPose.Pos().Distance(rightMarkerPose.Pos());
}

/////////////////////////////////////////////////
NavigationScoringPlugin::GateState NavigationScoringPlugin::Gate::IsPoseInGate(
    const ignition::math::Pose3d &_robotWorldPose) const
{
  // Transform to gate frame.
  const ignition::math::Vector3d robotLocalPosition =
    this->pose.Rot().Inverse().RotateVector(_robotWorldPose.Pos() -
    this->pose.Pos());

  // Are we within the width?
  if (fabs(robotLocalPosition.Y()) <= this->width / 2.0)
  {
    if (robotLocalPosition.X() >= 0.0)
      return GateState::VEHICLE_AFTER;
    else
      return GateState::VEHICLE_BEFORE;
  }
  else
    return GateState::VEHICLE_OUTSIDE;
}

/////////////////////////////////////////////////
NavigationScoringPlugin::NavigationScoringPlugin()
{
  gzmsg << "Navigation scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void NavigationScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);

  // This is a required element.
  if (!_sdf->HasElement("course_name"))
  {
    gzerr << "Unable to find <course_name> element in SDF." << std::endl;
    return;
  }
#if GAZEBO_MAJOR_VERSION >= 8
  this->course =
    this->world->ModelByName(_sdf->Get<std::string>("course_name"));
#else
  this->course =
    this->world->GetModel(_sdf->Get<std::string>("course_name"));
#endif
  if (!this->course)
  {
    gzerr << "could not find " <<
      _sdf->Get<std::string>("course_name") << std::endl;
  }

  // Optional.
  if (_sdf->HasElement("obstacle_penalty"))
    this->obstaclePenalty = _sdf->Get<double>("obstacle_penalty");

  // This is a required element.
  if (!_sdf->HasElement("gates"))
  {
    gzerr << "Unable to find <gates> element in SDF." << std::endl;
    return;
  }

  // Parse all the gates.
  auto const &gatesElem = _sdf->GetElement("gates");
  if (!this->ParseGates(gatesElem))
  {
    gzerr << "Score has been disabled" << std::endl;
    return;
  }

  // Save number of gates
  this->numGates = this->gates.size();

  // Set default score in case of timeout.
  double timeoutScore = 2.0 * this->GetRunningStateDuration() /
                        static_cast<double>(this->numGates);
  gzmsg << "Setting timeoutScore = " << timeoutScore << std::endl;
  this->ScoringPlugin::SetTimeoutScore(timeoutScore);

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&NavigationScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
bool NavigationScoringPlugin::ParseGates(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "NavigationScoringPlugin::ParseGates(): NULL _sdf pointer");

  // We need at least one gate.
  if (!_sdf->HasElement("gate"))
  {
    gzerr << "Unable to find <gate> element in SDF." << std::endl;
    return false;
  }

  auto gateElem = _sdf->GetElement("gate");

  // Parse a new gate.
  while (gateElem)
  {
    // The left marker's name.
    if (!gateElem->HasElement("left_marker"))
    {
      gzerr << "Unable to find <left_marker> element in SDF." << std::endl;
      return false;
    }

    const std::string leftMarkerName =
      gateElem->Get<std::string>("left_marker");

    // The right marker's name.
    if (!gateElem->HasElement("right_marker"))
    {
      gzerr << "Unable to find <right_marker> element in SDF." << std::endl;
      return false;
    }

    const std::string rightMarkerName =
      gateElem->Get<std::string>("right_marker");

    if (!this->AddGate(leftMarkerName, rightMarkerName))
      return false;

    // Parse the next gate.
    gateElem = gateElem->GetNextElement("gate");
  }

  return true;
}

//////////////////////////////////////////////////
bool NavigationScoringPlugin::AddGate(const std::string &_leftMarkerName,
    const std::string &_rightMarkerName)
{
  gazebo::physics::LinkPtr leftMarkerModel =
    this->course->GetLink(this->course->GetName() + "::" +
      _leftMarkerName + "::link");

  // Sanity check: Make sure that the model exists.
  if (!leftMarkerModel)
  {
    gzerr << "Unable to find model [" << _leftMarkerName << "]" << std::endl;
    return false;
  }

  gazebo::physics::LinkPtr rightMarkerModel =
    this->course->GetLink(this->course->GetName() + "::" +
      _rightMarkerName + "::link");

  // Sanity check: Make sure that the model exists.
  if (!rightMarkerModel)
  {
    gzerr << "Unable to find model [" << _rightMarkerName << "]" << std::endl;
    return false;
  }

  // Save the new gate.
  this->gates.push_back(Gate(leftMarkerModel, rightMarkerModel));

  return true;
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::Update()
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

  // Skip if we're not in running mode.
  if (this->TaskState() != "running")
    return;

  // Current score
  this->ScoringPlugin::SetScore(std::min(this->GetRunningStateDuration(),
    this->ElapsedTime().Double() +
    this->numCollisions * this->obstaclePenalty)/this->numGates);

#if GAZEBO_MAJOR_VERSION >= 8
  const auto robotPose = this->vehicleModel->WorldPose();
#else
  const auto robotPose = this->vehicleModel->GetWorldPose().Ign();
#endif

  // Update the state of all gates.
  auto iter = std::begin(this->gates);
  while (iter != std::end(this->gates))
  {
    Gate &gate = *iter;

    // Update this gate (in case it moved).
    gate.Update();

    // Check if we have crossed this gate.
    auto currentState = gate.IsPoseInGate(robotPose);
    if (currentState == GateState::VEHICLE_AFTER &&
        gate.state   == GateState::VEHICLE_BEFORE)
    {
      currentState = GateState::CROSSED;
      gzmsg << "New gate crossed!" << std::endl;

      // We need to cross all gates in order.
      if (iter != this->gates.begin())
      {
        gzmsg << "Gate crossed in the wrong order" << std::endl;
        this->Fail();
        return;
      }

      iter = this->gates.erase(iter);
    }
    // Just checking: did we go backward through the gate?
    else if (currentState == GateState::VEHICLE_BEFORE &&
             gate.state   == GateState::VEHICLE_AFTER)
    {
      currentState = GateState::INVALID;
      gzmsg << "Transited the gate in the wrong direction. Gate invalidated!"
            << std::endl;
      this->Fail();
      return;
    }
    else
      ++iter;

    gate.state = currentState;
  }

  // Course completed!
  if (this->gates.empty())
  {
    gzmsg << "Course completed!" << std::endl;
    this->Finish();
  }
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::Fail()
{
  this->SetScore(this->ScoringPlugin::GetTimeoutScore());
  this->Finish();
}

//////////////////////////////////////////////////
void NavigationScoringPlugin::OnCollision()
{
  this->numCollisions++;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(NavigationScoringPlugin)
