/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/KeyEventHandler.hh>

#include "TrafficLightsGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(TrafficLightsGUIPlugin)

/////////////////////////////////////////////////
TrafficLightsGUIPlugin::TrafficLightsGUIPlugin()
  : GUIPlugin()
{
  auto mainFrame = new QFrame();

  auto mainLayout = new QHBoxLayout;
  mainLayout->addWidget(mainFrame);

  this->setLayout(mainLayout);
  this->resize(1, 1);
}

/////////////////////////////////////////////////
void TrafficLightsGUIPlugin::Load(sdf::ElementPtr _sdf)
{
  // Get input params from SDF
  if (!_sdf->HasElement("key"))
    return;

  sdf::ElementPtr keyElem = _sdf->GetElement("key");

  while (keyElem)
  {
    Key key;
    key.value = keyElem->Get<int>("value");
    key.model = keyElem->Get<std::string>("model");
    key.color = keyElem->Get<std::string>("color");

    this->keys.push_back(key);
    keyElem = keyElem->GetNextElement("key");
  }

  // Initialize transport
  this->node = gazebo::transport::NodePtr(
      new gazebo::transport::Node());
  this->node->Init();

  this->keyboardSub =
      this->node->Subscribe("/gazebo/default/keyboard/keypress",
      &TrafficLightsGUIPlugin::OnKeyPress, this, true);

  // Publish to visuals
  this->visPub =
    this->node->Advertise<gazebo::msgs::Visual>("/gazebo/default/visual");
}

/////////////////////////////////////////////////
void TrafficLightsGUIPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  // Colors
  auto red = std::make_pair(gazebo::common::Color::Red, "red");
  auto yellow = std::make_pair(gazebo::common::Color::Yellow, "yellow");
  auto green = std::make_pair(gazebo::common::Color::Green, "green");
  auto black = gazebo::common::Color::Black;

  for (auto key : this->keys)
  {
    if (_msg->int_value() == key.value)
    {
      auto color = key.color;
      auto model = key.model;

      for (auto c : {red, yellow, green})
      {
        for (auto l : {"right_light", "center_light"})
        {
          gazebo::msgs::Visual msg;
          msg.set_type(gazebo::msgs::Visual::VISUAL);
          msg.set_parent_name(model + "::" + l + "::link");

          msg.set_name(model + "::" + l + "::link::" + c.second);

          auto matMsg = msg.mutable_material();
          if (c.second == color)
          {
            gazebo::msgs::Set(matMsg->mutable_emissive(), c.first);
            gazebo::msgs::Set(matMsg->mutable_ambient(), c.first);
          }
          else
          {
            gazebo::msgs::Set(matMsg->mutable_emissive(), black);
            gazebo::msgs::Set(matMsg->mutable_ambient(), black);
          }

          this->visPub->Publish(msg);
        }
      }
    }
  }
}

