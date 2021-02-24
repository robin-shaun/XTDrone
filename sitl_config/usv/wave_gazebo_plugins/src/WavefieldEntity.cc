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

#include <iostream>
#include <string>

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector2.hh>

#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/Utilities.hh"

using namespace gazebo;
using namespace common;

namespace asv
{

/////////////////////////////////////////////////////////////////////////////
// WavefieldEntity

  /// \internal
  /// \brief Private data for the WavefieldEntity
  class WavefieldEntityPrivate
  {
    /// \brief The size of the wavefield. Default value is [1000 1000].
    public: ignition::math::Vector2d size;

    /// \brief The number of grid cells in the wavefield.
    /// Default value is [50 50].
    public: ignition::math::Vector2d cellCount;

    /// \brief The wave parameters.
    public: std::shared_ptr<asv::WaveParameters> waveParams;
  };

  /////////////////////////////////////////////////
  WavefieldEntity::~WavefieldEntity()
  {
  }

  /////////////////////////////////////////////////
  WavefieldEntity::WavefieldEntity(physics::BasePtr _parent) :
    Base(_parent),
    data(new WavefieldEntityPrivate())
  {
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Load(sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    Base::Load(_sdf);

    // Wavefield Parameters
    this->data->size = Utilities::SdfParamVector2(*_sdf, "size", \
                                  ignition::math::Vector2d(1000, 1000));
    this->data->cellCount = Utilities::SdfParamVector2(*_sdf, "cell_count", \
                                   ignition::math::Vector2d(50, 50));

    // Wave Parameters
    gzmsg << "WavefieldEntity: Loading WaveParameters from SDF" <<  std::endl;
    this->data->waveParams.reset(new WaveParameters());
    if (_sdf->HasElement("wave"))
    {
      gzmsg << "Found <wave> tag" << std::endl;
      sdf::ElementPtr sdfWave = _sdf->GetElement("wave");
      this->data->waveParams->SetFromSDF(*sdfWave);
    }
    else
    {
      gzmsg << "Missing <wave> tag" << std::endl;
    }
    // @DEBUG_INFO
    this->data->waveParams->DebugPrint();
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Fini()
  {
    Base::Fini();
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Init()
  {
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Reset()
  {
  }

  /////////////////////////////////////////////////
  void WavefieldEntity::Update()
  {
  }

  /////////////////////////////////////////////////
  std::shared_ptr<const WaveParameters> WavefieldEntity::GetWaveParams() const
  {
    return this->data->waveParams;
  }

  /////////////////////////////////////////////////
  std::string WavefieldEntity::MakeName(const std::string& _parentName)
  {
    return std::string(_parentName + "::wavefield_entity");
  }
}
