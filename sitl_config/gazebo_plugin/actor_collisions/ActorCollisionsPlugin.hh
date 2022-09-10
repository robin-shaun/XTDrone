/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GAZEBO_EXAMPLES_PLUGINS_ACTORCOLLISIONSPLUGIN_HH_
#define GAZEBO_EXAMPLES_PLUGINS_ACTORCOLLISIONSPLUGIN_HH_

#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  /// \brief This plugin enables collisions on an Actor, and
  /// optionally applies a scaling factor and pose offset to each of the
  /// Actor's box collisions.
  /// Collisions only work on enabled objects. ODE has a auto-disable
  /// feature that "disables" an object when it comes to rest. An actor will
  /// pass through these objects.
  ///
  /// See actor_collisions.world for an example world file.
  ///
  class ActorCollisionsPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorCollisionsPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  };
}
#endif