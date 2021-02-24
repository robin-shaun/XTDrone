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

/// \file WavefieldEntity.hh
/// \brief This file contains the definition for a Gazebo physics object
/// that allows a wave field to be added into a simulated world.

#ifndef _WAVE_GAZEBO_PLUGINS_WAVEFIELD_ENTITY_HH_
#define _WAVE_GAZEBO_PLUGINS_WAVEFIELD_ENTITY_HH_

#include <memory>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Base.hh>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// WavefieldEntity

  class WaveParameters;

  /// \internal
  /// \brief Class to hold private data for WavefieldEntity.
  class WavefieldEntityPrivate;

  /// \brief A class to manage a wave field that can be accessed from the World.
  class GZ_PHYSICS_VISIBLE WavefieldEntity : public gazebo::physics::Base
  {
    /// \brief Destructor.
    public: virtual ~WavefieldEntity();

    /// \brief Constructor.
    public: explicit WavefieldEntity(gazebo::physics::BasePtr _parent);

    /// \brief Load.
    public: virtual void Load(sdf::ElementPtr _sdf);

    /// \brief Finialize the object.
    public: virtual void Fini();

    /// \brief Initialize the object.
    public: virtual void Init();

    /// \brief Reset the object.
    public: virtual void Reset();

    /// \brief Update the object.
    public: virtual void Update();

    /// \brief Get a pointer to the wave pararameters.
    std::shared_ptr<const WaveParameters> GetWaveParams() const;

    /// \brief Make a wave field entity name given a parent object name.
    ///
    /// \param[in] _parentName  The name of the parent object.
    /// \return                 The name of the wave field entity.
    public: static std::string MakeName(const std::string& _parentName);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WavefieldEntityPrivate> data;
  };
}

#endif  // _WAVE_GAZEBO_PLUGINS_WAVEFIELD_ENTITY_HH_
