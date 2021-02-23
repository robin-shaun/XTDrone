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

#include <algorithm>
#include <iostream>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include "wave_gazebo_plugins/Utilities.hh"

namespace asv
{
  /////////////////////////////////////////////////////////////////////////////
  // Templates

  // This code adapted vmrc/usv_gazebo_plugins/usv_gazebo_dynamics_plugin.cc
  template <typename T>T
    SdfParam(sdf::Element& _sdf, const std::string &_paramName, \
             const T &_defaultVal)
  {
    if (!_sdf.HasElement(_paramName))
    {
      gzmsg << "Parameter <" << _paramName << "> not found: "
        <<  "Using default value of <" << _defaultVal << ">." << std::endl;
      return _defaultVal;
    }

    T val = _sdf.Get<T>(_paramName);
    gzmsg << "Parameter found - setting <" << _paramName
      << "> to <" << val << ">." << std::endl;
    return val;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Utilities
  bool Utilities::SdfParamBool(sdf::Element& _sdf,
    const std::string& _paramName, const bool _defaultVal)
  {
    return SdfParam<bool>(_sdf, _paramName, _defaultVal);
  }

  size_t Utilities::SdfParamSizeT(sdf::Element& _sdf,
    const std::string& _paramName, const size_t _defaultVal)
  {
    return SdfParam<double>(_sdf, _paramName, _defaultVal);
  }

  double Utilities::SdfParamDouble(sdf::Element& _sdf,
    const std::string& _paramName, const double _defaultVal)
  {
    return SdfParam<double>(_sdf, _paramName, _defaultVal);
  }

  std::string Utilities::SdfParamString(sdf::Element& _sdf,
    const std::string& _paramName, const std::string &_defaultVal)
  {
    return SdfParam<std::string>(_sdf, _paramName, _defaultVal);
  }

  ignition::math::Vector2d Utilities::SdfParamVector2(sdf::Element& _sdf,
    const std::string& _paramName, const ignition::math::Vector2d _defaultVal)
  {
    return SdfParam<ignition::math::Vector2d>(_sdf, _paramName, _defaultVal);
  }

  ignition::math::Vector3d Utilities::SdfParamVector3(sdf::Element& _sdf,
    const std::string& _paramName, const ignition::math::Vector3d _defaultVal)
  {
    return SdfParam<ignition::math::Vector3d>(_sdf, _paramName, _defaultVal);
  }
}
