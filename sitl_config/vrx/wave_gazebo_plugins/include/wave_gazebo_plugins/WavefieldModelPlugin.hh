/*
 * Copyright (C) 2019  Rhys Mainwaring
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

/// \file WavefieldModelPlugin.hh
/// \brief This file defines a Gazebo ModelPlugin used to manage a wave field.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_

#include <memory>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// WavefieldModelPlugin

  // Forward declarations
  class WavefieldModelPluginPrivate;
  class WaveParameters;

  /// \brief A Gazebo model plugin to simulate water waves.
  ///
  /// # Usage
  ///
  /// Add the SDF for the plugin to the <model> element of your wave model.
  ///
  /// \code
  /// <plugin name="wavefield" filename="libWavefieldModelPlugin.so">
  ///   <static>false</static>
  ///   <update_rate>30</update_rate>
  ///   <size>1000 1000</size>
  ///   <cell_count>50 50</cell_count>
  ///   <wave>
  ///     <number>3</number>
  ///     <scale>1.5</scale>
  ///     <angle>0.4</angle>
  ///     <steepness>1.0</steepness>
  ///     <amplitude>0.4</amplitude>
  ///     <period>8.0</period>
  ///     <direction>1 1</direction>
  ///   </wave>
  /// </plugin>
  /// \endcode
  ///
  /// # Parameters
  ///
  /// 1. <static> (bool, default: false)
  ///   Create a static wave field if set to true.
  ///
  /// 2. <update_rate> (double, default: 30.0)
  ///   The rate in Hz at which the wavefield is updated.
  ///
  /// 3. <size> (Vector2D, default: (1000 1000))
  ///   A two component vector for the size of the wave field in each direction.
  ///
  /// 4. <cell_count> (int, default: (50 50))
  ///   A two component vector for the number of grid cells in each direction.
  ///
  /// 5. <number> (int, default: 1)
  ///   The number of component waves.
  ///
  /// 6. <scale> (double, default: 2.0)
  ///   The scale between the mean and largest / smallest component waves.
  ///
  /// 7. <angle> (double, default: 2*pi/10)
  ///   The angle between the mean wave direction and the
  ///   largest / smallest component waves.
  ///
  /// 8. <steepness> (double, default: 1.0)
  ///   A parameter in [0, 1] controlling the wave steepness with
  ///   1 being steepest.
  ///
  /// 9. <amplitude> (double, default: 0.0)
  ///   The amplitude of the mean wave in [m].
  ///
  /// 10. <period> (double, default: 1.0)
  ///   The period of the mean wave in [s].
  ///
  /// 11. <phase> (double, default: 0.0)
  ///   The phase of the mean wave.
  ///
  /// 12. <direction> (Vector2D, default: (1 0))
  ///   A two component vector specifiying the direction of the mean wave.
  ///
  /// 13. <model> (string, default: default)
  ///   The model used to generate component waves.
  ///   Should be either "PMS" or "CWR"
  ///
  /// 14. <gain> (double, default: 1.0)
  ///   For PMS, the multiplier applied to component amplitudes.
  ///
  /// 15. <tau> (double, default: 1.0)
  ///   Time constant used to gradually increase wavefield at startup.
  ///
  class GAZEBO_VISIBLE WavefieldModelPlugin : public gazebo::ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~WavefieldModelPlugin();

    /// \brief Constructor.
    public: WavefieldModelPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: void Init();

    // Documentation inherited.
    public: void Fini();

    // Documentation inherited.
    public: void Reset();

    /// \brief Retrive a pointer to the wavefield parameters
    /// from the Wavefield plugin.
    ///
    /// \param _world   A pointer to the world containing the wave field.
    /// \param _waveModelName   The name of the wavefield model
    ///                         containing the wave field.
    /// \return A valid pointer to WaveParameters if found and nullptr if not.
    public: static std::shared_ptr<const WaveParameters> GetWaveParams(
      gazebo::physics::WorldPtr _world,
      const std::string& _waveModelName);

    /// internal
    /// \brief Callback for World Update events.
    private: void OnUpdate();
    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldModelPluginPrivate> data;
  };
}

#endif  // _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_MODEL_PLUGIN_HH_
