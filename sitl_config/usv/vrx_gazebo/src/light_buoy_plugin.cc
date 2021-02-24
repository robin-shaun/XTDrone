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
#include <gazebo/rendering/Scene.hh>
#include "vrx_gazebo/light_buoy_plugin.hh"

const std::array<LightBuoyPlugin::Colors_t, 5> LightBuoyPlugin::kColors
  = {LightBuoyPlugin::Colors_t(CreateColor(1.0, 0.0, 0.0, 1.0), "red"),
     LightBuoyPlugin::Colors_t(CreateColor(0.0, 1.0, 0.0, 1.0), "green"),
     LightBuoyPlugin::Colors_t(CreateColor(0.0, 0.0, 1.0, 1.0), "blue"),
     LightBuoyPlugin::Colors_t(CreateColor(1.0, 1.0, 0.0, 1.0), "yellow"),
     LightBuoyPlugin::Colors_t(CreateColor(0.0, 0.0, 0.0, 1.0), "off")};

//////////////////////////////////////////////////
std_msgs::ColorRGBA LightBuoyPlugin::CreateColor(const double _r,
  const double _g, const double _b, const double _a)
{
  static std_msgs::ColorRGBA color;
  color.r = _r;
  color.g = _g;
  color.b = _b;
  color.a = _a;
  return color;
}

//////////////////////////////////////////////////
uint8_t LightBuoyPlugin::IndexFromColor(const std::string &_color)
{
  uint8_t index = 0u;
  for (auto color : kColors)
  {
    if (_color == color.second)
      return index;

    ++index;
  }

  return std::numeric_limits<uint8_t>::max();
}

//////////////////////////////////////////////////
void LightBuoyPlugin::InitializeAllPatterns()
{
  for (uint8_t i = 0u; i < this->kColors.size() - 1; ++i)
  {
    for (uint8_t j = 0u; j < this->kColors.size() - 1; ++j)
    {
      if (j == i)
        continue;

      for (uint8_t k = 0u; k < this->kColors.size() - 1; ++k)
      {
        if (k == j)
          continue;

        // The last two colors are always OFF.
        this->allPatterns.push_back({i, j, k,
          this->IndexFromColor("off"), this->IndexFromColor("off")});
      }
    }
  }
}

//////////////////////////////////////////////////
LightBuoyPlugin::LightBuoyPlugin() :
  gzNode(new gazebo::transport::Node())
{
}

//////////////////////////////////////////////////
void LightBuoyPlugin::ChangePatternTo(
  const gazebo::ConstLightBuoyColorsPtr &_msg)
{
  pattern[0] = IndexFromColor(_msg->color_1());
  pattern[1] = IndexFromColor(_msg->color_2());
  pattern[2] = IndexFromColor(_msg->color_3());
  pattern[3] = IndexFromColor("off");
  pattern[4] = IndexFromColor("off");

  // If we get a new pattern, reinitialize the sequence.
  std::lock_guard<std::mutex> lock(this->mutex);
  this->nextUpdateTime = this->scene->SimTime() + gazebo::common::Time(1.0);
  this->state = 0;

  return;
}

//////////////////////////////////////////////////
void LightBuoyPlugin::Load(gazebo::rendering::VisualPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  this->scene = _parent->GetScene();

  GZ_ASSERT(this->scene != nullptr, "NULL scene");

  this->InitializeAllPatterns();

  if (!this->ParseSDF(_sdf))
    return;

  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  if (this->shuffleEnabled)
  {
    this->nh = ros::NodeHandle(this->ns);
    this->changePatternSub = this->nh.subscribe(
      this->rosShuffleTopic, 1, &LightBuoyPlugin::ChangePattern, this);
  }

  this->nextUpdateTime = this->scene->SimTime();

  this->updateConnection = gazebo::event::Events::ConnectPreRender(
    std::bind(&LightBuoyPlugin::Update, this));

  gzNode->Init();
  this->colorSub = this->gzNode->Subscribe
    (this->gzColorsTopic, &LightBuoyPlugin::ChangePatternTo, this);
}

//////////////////////////////////////////////////
bool LightBuoyPlugin::ParseSDF(sdf::ElementPtr _sdf)
{
  // Required: Sequence of colors.
  uint8_t i = 0u;
  for (auto colorIndex : {"color_1", "color_2", "color_3"})
  {
    if (!_sdf->HasElement(colorIndex))
    {
      ROS_ERROR("<%s> missing", colorIndex);
      return false;
    }

    auto color = _sdf->GetElement(colorIndex)->Get<std::string>();
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);

    // Sanity check: color should be red, green, blue, yellow or off.
    if (color != "red"  && color != "green" &&
        color != "blue" && color != "yellow" && color != "off")
    {
      ROS_ERROR("Invalid color [%s]", color.c_str());
      return false;
    }

    this->pattern[i++] = IndexFromColor(color);
  }

  // The last two colors of the pattern are always black.
  this->pattern[3] = IndexFromColor("off");
  this->pattern[4] = IndexFromColor("off");

  // Required: visuals.
  if (!_sdf->HasElement("visuals"))
  {
    ROS_ERROR("<visuals> missing");
    return false;
  }

  auto visualsElem = _sdf->GetElement("visuals");
  if (!visualsElem->HasElement("visual"))
  {
    ROS_ERROR("<visual> missing");
    return false;
  }

  auto visualElem = visualsElem->GetElement("visual");
  while (visualElem)
  {
    std::string visualName = visualElem->Get<std::string>();
    this->visualNames.push_back(visualName);
    visualElem = visualElem->GetNextElement();
  }

  // Optional: Is shuffle enabled?
  if (_sdf->HasElement("shuffle"))
  {
    this->shuffleEnabled = _sdf->GetElement("shuffle")->Get<bool>();

    // Required if shuffle enabled: ROS topic.
    if (!_sdf->HasElement("ros_shuffle_topic"))
    {
      ROS_ERROR("<ros_shuffle_topic> missing");
    }
    this->rosShuffleTopic = _sdf->GetElement
      ("ros_shuffle_topic")->Get<std::string>();
  }

  // optional gzColorsTopic
  if (!_sdf->HasElement("gz_colors_topic"))
  {
    this->gzColorsTopic = "/vrx/light_buoy/new_pattern";
  }
  else
  {
    this->gzColorsTopic = _sdf->GetElement
      ("gz_colors_topic")->Get<std::string>();
  }
  // Optional: ROS namespace.
  if (_sdf->HasElement("robot_namespace"))
    this->ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  return true;
}

//////////////////////////////////////////////////
void LightBuoyPlugin::Update()
{
  // Get the visuals if needed.
  if (this->visuals.empty())
  {
    for (auto visualName : this->visualNames)
    {
      auto visualPtr = this->scene->GetVisual(visualName);
      if (visualPtr)
        this->visuals.push_back(visualPtr);
      else
        ROS_ERROR("Unable to find [%s] visual", visualName.c_str());
    }
  }

  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->scene->SimTime() < this->nextUpdateTime)
    return;

  this->nextUpdateTime = this->nextUpdateTime + gazebo::common::Time(1.0);

  // Start over if at end of pattern
  if (this->state > 4)
    this->state = 0;

  auto color = this->kColors[this->pattern[this->state]].first;
  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Color gazeboColor(color.r, color.g, color.b, color.a);
  #else
    gazebo::common::Color gazeboColor(color.r, color.g, color.b, color.a);
  #endif
  // Update the visuals.
  for (auto visual : this->visuals)
  {
    visual->SetAmbient(gazeboColor);
    visual->SetDiffuse(gazeboColor);
  }

  // Increment index for next timer callback
  ++this->state;
}

//////////////////////////////////////////////////
void LightBuoyPlugin::ChangePattern(const std_msgs::Empty::ConstPtr &_msg)
{
  this->pattern = this->allPatterns[this->allPatternsIdx];
  this->allPatternsIdx = (this->allPatternsIdx + 1) % this->allPatterns.size();

  // Generate string representing pattern, ex: "RGB"
  std::string colorSeq = "";
  for (size_t i = 0; i < 3; ++i)
    colorSeq += this->kColors[this->pattern[i]].second[0];
  // Log the new pattern
  ROS_INFO_NAMED("LightBuoyPlugin", "Pattern is %s", colorSeq.c_str());
}

// Register plugin with gazebo
GZ_REGISTER_VISUAL_PLUGIN(LightBuoyPlugin)
