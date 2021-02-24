/*
 * Copyright (C) 2019 Brian Bingham
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

#ifndef WAVE_GAZEBO_PLUGINS_WAVEGUAGE_PLUGIN_HH_
#define WAVE_GAZEBO_PLUGINS_WAVEGUAGE_PLUGIN_HH_

#include <map>
#include <string>
#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A plugin that sets the model height (z) to be the same
  /// as the wave height calculated for the physics.
  /// All SDF parameters are optional.
  ///
  ///   <fluid_level>:   The height of the fluid/air interface [m].
  ///                    This parameter is optional.
  ///
  ///   <wave_model>:    Name of the model that includes and instance of the
  ///                    WavefieldModelPlugin.
  ///   For Example:
  ///   <plugin name="wavegauge_plugin" filename="libwavegauge_plugin.so">
  ///     <wave_model>ocean_waves</wave_model>
  ///     <fluid_level>0.0</fluid_level>
  ///   </plugin> example
  ///
  ///
  class WaveguagePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: WaveguagePlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model
    protected: physics::ModelPtr model;

    /// \brief The name of the wave model
    protected: std::string waveModelName;

    /// \brief The height of the fluid/air interface [m]. Defaults to 0.
    protected: double fluidLevel;
  };
}

#endif
