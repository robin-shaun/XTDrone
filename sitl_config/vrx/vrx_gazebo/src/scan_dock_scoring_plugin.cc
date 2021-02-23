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
#include <cmath>
#include <gazebo/common/Console.hh>
#include "vrx_gazebo/scan_dock_scoring_plugin.hh"

/////////////////////////////////////////////////
ColorSequenceChecker::ColorSequenceChecker(
  const std::vector<std::string> &_expectedColors,
  const std::string &_rosNameSpace, const std::string &_rosColorSequenceService)
  : expectedSequence(_expectedColors),
    ns(_rosNameSpace),
    colorSequenceService(_rosColorSequenceService)
{
  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }
  this->nh = ros::NodeHandle(this->ns);
}

/////////////////////////////////////////////////
void ColorSequenceChecker::Enable()
{
  this->colorSequenceServer = this->nh.advertiseService(
    this->colorSequenceService, &ColorSequenceChecker::OnColorSequence, this);
}

/////////////////////////////////////////////////
void ColorSequenceChecker::Disable()
{
  this->colorSequenceServer.shutdown();
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::SubmissionReceived() const
{
  return this->colorSequenceReceived;
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::Correct() const
{
  return this->correctSequence;
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::OnColorSequence(
  ros::ServiceEvent<vrx_gazebo::ColorSequence::Request,
    vrx_gazebo::ColorSequence::Response> &_event)
{
  ROS_INFO_NAMED("ColorSequenceChecker", "Color sequence submission received");

  const vrx_gazebo::ColorSequence::Request &req = _event.getRequest();
  vrx_gazebo::ColorSequence::Response &res = _event.getResponse();

  {
    // Sanity check: Only one color sequence submission is allowed.
    if (this->colorSequenceReceived)
    {
      ROS_ERROR("The color sequence has already been submitted");
      res.success = false;
      return false;
    }

    this->colorSequenceReceived = true;
  }

  // Sanity check: Make sure that we have the expected color sequence.
  if (this->expectedSequence.size() != 3u)
  {
    ROS_ERROR("The color sequence is not of size 3 - will be ignored.");
    res.success = false;
    return false;
  }

  std::string color1 = req.color1;
  std::string color2 = req.color2;
  std::string color3 = req.color3;

  std::transform(color1.begin(), color1.end(), color1.begin(), ::tolower);
  std::transform(color2.begin(), color2.end(), color2.begin(), ::tolower);
  std::transform(color3.begin(), color3.end(), color3.begin(), ::tolower);

  // Incorrect color sequence.
  this->correctSequence =
      color1 == this->expectedSequence[0] &&
      color2 == this->expectedSequence[1] &&
      color3 == this->expectedSequence[2];
  if (this->correctSequence)
  {
    ROS_INFO_NAMED("ColorSequenceChecker", "Received color sequence is "
      "correct.  Additional points will be scored.");
  }
  else
  {
    ROS_INFO_NAMED("ColorSequenceChecker", "Received color sequence is "
      "not correct. No additional points.");
  }

  // The submission is considered correct even if the sequence is wrong.
  res.success = true;
  return true;
}

/////////////////////////////////////////////////
DockChecker::DockChecker(const std::string &_name,
  const std::string &_internalActivationTopic,
  const std::string &_externalActivationTopic,
  const double _minDockTime,
  const bool _dockAllowed, const std::string &_worldName,
  const std::string &_rosNameSpace, const std::string &_announceSymbol,
  const std::string &_gzSymbolTopic)
  : name(_name),
    internalActivationTopic(_internalActivationTopic),
    externalActivationTopic(_externalActivationTopic),
    minDockTime(_minDockTime),
    dockAllowed(_dockAllowed),
    ns(_rosNameSpace),
    gzSymbolTopic(_gzSymbolTopic)
{
  this->timer.Stop();
  this->timer.Reset();

  this->announceSymbol.data = _announceSymbol;

  this->node.reset(new gazebo::transport::Node());
  this->node->Init();

  // Subscriber to receive ContainPlugin updates.
#if GAZEBO_MAJOR_VERSION >= 8
  this->ignNode.Subscribe(this->internalActivationTopic,
    &DockChecker::OnInternalActivationEvent, this);
  this->ignNode.Subscribe(this->externalActivationTopic,
    &DockChecker::OnExternalActivationEvent, this);
#else
  this->containSub = this->node->Subscribe(this->internalActivationTopic,
    &DockChecker::OnInternalActivationEvent, this);
  this->containSub = this->node->Subscribe(this->externalActivationTopic,
    &DockChecker::OnExternalActivationEvent, this);
#endif
}

/////////////////////////////////////////////////
bool DockChecker::AnytimeDocked() const
{
  return this->anytimeDocked;
}

/////////////////////////////////////////////////
bool DockChecker::AtEntrance() const
{
  return this->atEntrance;
}

/////////////////////////////////////////////////
bool DockChecker::Allowed() const
{
  return this->dockAllowed;
}

/////////////////////////////////////////////////
void DockChecker::AnnounceSymbol()
{
  // Override the docks own sdf parameters
  this->dockPlacardPub = this->node->Advertise
    <dock_placard_msgs::msgs::DockPlacard>(gzSymbolTopic);
  dock_placard_msgs::msgs::DockPlacard symbol;
  symbol.set_color(announceSymbol.data.substr
    (0, announceSymbol.data.find("_")));
  symbol.set_shape(announceSymbol.data.substr
    (announceSymbol.data.find("_")+1));
  this->dockPlacardPub->Publish(symbol);

  if (this->dockAllowed)
  {
    // Initialize ROS transport.
    this->nh.reset(new ros::NodeHandle());
    this->symbolPub =
      this->nh->advertise<std_msgs::String>(this->symbolTopic, 1, true);

    this->symbolPub.publish(this->announceSymbol);
  }
}

/////////////////////////////////////////////////
void DockChecker::Update()
{
  if (this->anytimeDocked)
    return;

  this->anytimeDocked =
    this->timer.GetElapsed() >= gazebo::common::Time(this->minDockTime);

  if (this->anytimeDocked)
  {
    gzmsg  << "Successfully stayed in dock for " << this->minDockTime
           << " seconds, transitioning to <docked> state" << std::endl;
  }
}

#if GAZEBO_MAJOR_VERSION >= 8
/////////////////////////////////////////////////
void DockChecker::OnInternalActivationEvent(const ignition::msgs::Boolean &_msg)
{
  // Currently docked.
  if (_msg.data() == 1)
  {
    this->timer.Start();
    gzmsg << "Entering internal dock activation zone, transitioning to "
          << "<docking> state in [" << this->name << "]." << std::endl;
  }

  // Currently undocked.
  if (_msg.data() == 0)
  {
    this->timer.Stop();
    this->timer.Reset();
    if (this->AnytimeDocked())
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] after required time - transitioning to <exited> state."
            << std::endl;
    }
    else
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] early - transitioning back to <undocked> state."
            << std::endl;
    }
  }

  gzdbg << "[" << this->name << "] OnInternalActivationEvent(): "
        << _msg.data() << std::endl;
}

/////////////////////////////////////////////////
void DockChecker::OnExternalActivationEvent(const ignition::msgs::Boolean &_msg)
{
  this->atEntrance = _msg.data() == 1;

  if (this->atEntrance)
  {
    gzmsg << "Entering external dock activation zone in [" << this->name
          << "]" << std::endl;
  }
  else
  {
    gzmsg << "Leaving external dock activation zone in [" << this->name
          << "]" << std::endl;
  }

  gzdbg << "[" << this->name << "] OnExternalActivationEvent(): "
        << _msg.data() << std::endl;
}
#else
/////////////////////////////////////////////////
void DockChecker::OnInternalActivationEvent(ConstIntPtr &_msg)
{
  // Currently docked.
  if (_msg->data() == 1)
  {
    this->timer.Start();
    gzmsg << "Entering internal dock activation zone, transitioning to "
          << "<docking> state in [" << this->name << "]." << std::endl;
  }

  // Currently undocked.
  if (_msg->data() == 0)
  {
    this->timer.Stop();
    this->timer.Reset();
    if (this->AnytimeDocked())
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] after required time - transitioning to <exited> state."
            << std::endl;
    }
    else
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] early - transitioning back to <undocked> state."
            << std::endl;
    }
  }

  gzdbg << "[" << this->name << "] OnInternalActivationEvent(): "
        << _msg->data() << std::endl;
}

/////////////////////////////////////////////////
void DockChecker::OnExternalActivationEvent(ConstIntPtr &_msg)
{
  this->atEntrance = _msg->data() == 1;

  if (this->atEntrance)
  {
    gzmsg << "Entering external dock activation zone in [" << this->name
          << "]" << std::endl;
  }
  else
  {
    gzmsg << "Leaving external dock activation zone in [" << this->name
          << "]" << std::endl;
  }

  gzdbg << "[" << this->name << "] OnExternalActivationEvent(): "
        << _msg->data() << std::endl;
}
#endif

//////////////////////////////////////////////////
ScanDockScoringPlugin::ScanDockScoringPlugin():
  node (new gazebo::transport::Node())
{
}

/////////////////////////////////////////////////
void ScanDockScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  this->node->Init();
  ScoringPlugin::Load(_world, _sdf);

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  if (!this->ParseSDF(_sdf))
    return;

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScanDockScoringPlugin::Update, this));

  this->lightBuoySequencePub = this->node->Advertise
    <light_buoy_colors_msgs::msgs::LightBuoyColors>(this->colorTopic);
}

//////////////////////////////////////////////////
bool ScanDockScoringPlugin::ParseSDF(sdf::ElementPtr _sdf)
{
  // Optional: ROS namespace.
  std::string ns;
  if (_sdf->HasElement("robot_namespace"))
    ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  // Enable color checker - default is true
  this->enableColorChecker = true;
  if (_sdf->HasElement("enable_color_checker"))
  {
    enableColorChecker =
      _sdf->GetElement("enable_color_checker")->Get<bool>();
  }
  // Optional: ROS service.
  std::string colorSequenceService = "/vrx/scan_dock/color_sequence";
  if (_sdf->HasElement("color_sequence_service"))
  {
    colorSequenceService =
      _sdf->GetElement("color_sequence_service")->Get<std::string>();
  }

  // Required: The expected color pattern.
  for (auto colorIndex : {"color_1", "color_2", "color_3"})
  {
    if (!_sdf->HasElement(colorIndex))
    {
      ROS_ERROR("<%s> missing", colorIndex);
      return false;
    }

    auto color = _sdf->GetElement(colorIndex)->Get<std::string>();
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);

    // Sanity check: color should be red, green, blue or yellow.
    if (color != "red"  && color != "green" &&
        color != "blue" && color != "yellow")
    {
      ROS_ERROR("Invalid color [%s]", color.c_str());
      return false;
    }
    this->expectedSequence.push_back(color);
  }

  // Optional: gazebo topic where light buoy sequence is published
  if (!_sdf->HasElement("color_topic"))
  {
    this->colorTopic = "/vrx/light_buoy/new_pattern";
  }
  else
  {
    this->colorTopic = _sdf->GetElement("color_topic")->Get<std::string>();
  }

  // Optional: the points granted when reported the correct color sequence.
  if (_sdf->HasElement("color_bonus_points"))
  {
    this->colorBonusPoints =
      _sdf->GetElement("color_bonus_points")->Get<double>();
  }

  // Instantiate the color checker.
  if (this->enableColorChecker)
  {
    this->colorChecker.reset(
      new ColorSequenceChecker(this->expectedSequence, ns,
                                colorSequenceService));
  }

  // Required: Parse the bays.
  if (!_sdf->HasElement("bays"))
  {
    ROS_ERROR("<bays> missing");
    return false;
  }

  auto baysElem = _sdf->GetElement("bays");
  if (!baysElem->HasElement("bay"))
  {
    ROS_ERROR("<bay> missing");
    return false;
  }

  auto bayElem = baysElem->GetElement("bay");
  while (bayElem)
  {
    // Required: bay name.
    if (!bayElem->GetElement("name"))
    {
      ROS_ERROR("<bays::bay::name> missing");
      return false;
    }
    std::string bayName = bayElem->Get<std::string>("name");

    // Required: internal_activation topic.
    if (!bayElem->GetElement("internal_activation_topic"))
    {
      ROS_ERROR("<bays::bay::internal_activation_topic> missing");
      return false;
    }
    std::string internalActivationTopic =
      bayElem->Get<std::string>("internal_activation_topic");

    // Required: external_activation topic.
    if (!bayElem->GetElement("external_activation_topic"))
    {
      ROS_ERROR("<bays::bay::external_activation_topic> missing");
      return false;
    }
    std::string externalActivationTopic =
      bayElem->Get<std::string>("external_activation_topic");

    // Required: gazebo symbol topic.
    if (!bayElem->GetElement("symbol_topic"))
    {
      ROS_ERROR("<bays::bay::symbol_topic> missing");
      return false;
    }
    std::string symbolTopic = bayElem->Get<std::string>("symbol_topic");

    // Required: minimum time to be considered "docked".
    if (!bayElem->GetElement("min_dock_time"))
    {
      ROS_ERROR("<bays::bay::min_dock_time> missing");
      return false;
    }
    double minDockTime = bayElem->Get<double>("min_dock_time");

    // Required: dock allowed.
    if (!bayElem->GetElement("dock_allowed"))
    {
      ROS_ERROR("<bays::bay::dock_allowed> missing");
      return false;
    }
    bool dockAllowed = bayElem->Get<bool>("dock_allowed");

    std::string announceSymbol = "";
    if (!bayElem->HasElement("symbol"))
    {
      ROS_ERROR("<bays::bay::symbol> not found");
    }
    announceSymbol =
      bayElem->GetElement("symbol")->Get<std::string>();


    // Create a new dock checker.
    #if GAZEBO_MAJOR_VERSION >= 8
      std::unique_ptr<DockChecker> dockChecker(
        new DockChecker(bayName, internalActivationTopic,
          externalActivationTopic, minDockTime, dockAllowed,
          this->world->Name(), ns, announceSymbol, symbolTopic));
    #else
      std::unique_ptr<DockChecker> dockChecker(
        new DockChecker(bayName, internalActivationTopic,
          externalActivationTopic, minDockTime, dockAllowed,
          this->world->GetName(), ns, announceSymbol, symbolTopic));
    #endif

    // Add the dock checker.
    this->dockCheckers.push_back(std::move(dockChecker));

    // Process the next checker.
    bayElem = bayElem->GetNextElement();
  }

  // Optional: the points granted when the vehicle docks in any bay.
  if (_sdf->HasElement("dock_bonus_points"))
  {
    this->dockBonusPoints =
      _sdf->GetElement("dock_bonus_points")->Get<double>();
  }

  // Optional: the points granted when the vehicle docks in the expected bay.
  if (_sdf->HasElement("correct_dock_bonus_points"))
  {
    this->correctDockBonusPoints =
      _sdf->GetElement("correct_dock_bonus_points")->Get<double>();
  }

  return true;
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::Update()
{
  if (this->enableColorChecker)
  {
    // Verify the color checker.
    if (!this->colorSubmissionProcessed &&
        this->colorChecker->SubmissionReceived())
    {
      // We need to decide if we grant extra points.
      if (this->colorChecker->Correct())
      {
        gzmsg << "Adding <" << this->colorBonusPoints << "> points for correct "
          << "reporting of color sequence" << std::endl;
        this->SetScore(this->Score() + this->colorBonusPoints);
      }

      // We only allow one color sequence submission.
      this->colorChecker->Disable();
      this->colorSubmissionProcessed = true;
    }
  }

  // Verify the dock checkers.
  for (auto &dockChecker : this->dockCheckers)
  {
    // We always need to update the checkers.
    dockChecker->Update();

    // Nothing to do if nobody ever docked or we're still inside the bay.
    if (!dockChecker->AnytimeDocked() || !dockChecker->AtEntrance())
      continue;

    // Points granted for docking!
    this->SetScore(this->Score() + this->dockBonusPoints);
    if (this->TaskState() == "running")
    {
    gzmsg  << "Successfully docked in [" << dockChecker->name << "]"
      << ". Awarding " << this->dockBonusPoints << " points." <<std::endl;
    }

    // Is this the right bay?
    if (dockChecker->Allowed())
    {
      this->SetScore(this->Score() + this->correctDockBonusPoints);
      if (this->TaskState() == "running")
      {
        gzmsg << "Docked in correct dock [" << dockChecker->name << "]"
              << ". Awarding " << this->correctDockBonusPoints
              << " more points." << std::endl;
      }
    }
    else
    {
      if (this->TaskState() == "running")
      {
        gzmsg  << "Docked in incorrect dock [" << dockChecker->name << "]"
        << ". No additional points." <<std::endl;
      }
    }

    // Time to finish the task as the vehicle docked.
    // Note that we only allow to dock one time. This is to prevent teams
    // docking in all possible bays.
    this->Finish();
    break;
  }
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;
  // Announce the symbol if needed.
  for (auto &dockChecker : this->dockCheckers)
    dockChecker->AnnounceSymbol();
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
  light_buoy_colors_msgs::msgs::LightBuoyColors colors;
  colors.set_color_1(this->expectedSequence[0]);
  colors.set_color_2(this->expectedSequence[1]);
  colors.set_color_3(this->expectedSequence[2]);
  lightBuoySequencePub->Publish(colors);

  if (this->enableColorChecker)
  {
    this->colorChecker->Enable();
  }
  // Announce the symbol if needed.
  for (auto &dockChecker : this->dockCheckers)
    dockChecker->AnnounceSymbol();
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(ScanDockScoringPlugin)
