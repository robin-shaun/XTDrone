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
#ifndef CITYSIM_TRAFFICLIGHTS_GUI_PLUGIN_HH_
#define CITYSIM_TRAFFICLIGHTS_GUI_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
    struct Key
    {
      int value;
      std::string color;
      std::string model;
    };

    class GAZEBO_VISIBLE TrafficLightsGUIPlugin : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      public: TrafficLightsGUIPlugin();

      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: void OnKeyPress(ConstAnyPtr &_msg);

      /// \brief Node used to establish communication with gzserver.
      private: transport::NodePtr node;

      /// \brief Publisher of factory messages.
      private: transport::PublisherPtr visPub;
      private: transport::SubscriberPtr keyboardSub;

      private: std::vector<Key> keys;
    };
}
#endif
