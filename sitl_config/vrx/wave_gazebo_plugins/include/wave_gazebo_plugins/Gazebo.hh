/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

/// \file Gazebo.hh
/// \brief Support for methods not available in legacy versions of Gazebo.

#ifndef _WAVE_GAZEBO_PLUGINS_GAZEBO_HH_
#define _WAVE_GAZEBO_PLUGINS_GAZEBO_HH_

#include <string>

namespace gazebo
{
  namespace rendering
  {
    class Visual;

    /// \brief Set a shader program parameter associated to this visual's
    /// material
    /// \param[in] _visual Reference to a Visual
    /// \param[in] _paramName Name of shader parameter
    /// \param[in] _shaderType Type of shader. Supported types:
    /// vertex, fragment
    /// \param[in] _value Value to set the parameter to. The value string can
    /// be a number (int, float) or a space delimited array of numbers
    /// (floats). The value type must match the type defined in the shaders.
    /// Note: Setting vec2/float2 params is only supported in ogre1.9+
    void SetMaterialShaderParam(
      Visual& _visual,
      const std::string &_paramName,
      const std::string &_shaderType,
      const std::string &_value);
  };
}

#endif
