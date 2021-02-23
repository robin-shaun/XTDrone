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

/// \file Geometry.hh
/// \brief This file contains methods to calculate properties of
/// simple geometrical objects.

#ifndef _WAVE_GAZEBO_PLUGINS_GEOMETRY_HH_
#define _WAVE_GAZEBO_PLUGINS_GEOMETRY_HH_

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// Geometry

  /// \brief A collection of static methods concerning linear geometry.
  class Geometry
  {
    /// \brief Normalise a Vector2 (i.e. ensure it has unit length)
    ///
    /// \param[in] _v     The vector to normalise.
    /// \return           The normalized vector.
    public: static ignition::math::Vector2d
    Normalize(const ignition::math::Vector2d& _v);

    /// \brief Normalise a Vector3 (i.e. ensure it has unit length)
    ///
    /// \param[in] _v     The vector to normalise.
    /// \return           The normalized vector.
    public: static ignition::math::Vector3d
    Normalize(const ignition::math::Vector3d& _v);

    /// \brief Compute the (normalised) normal to the plane defined
    /// by a triangle.
    ///
    /// \param[in] _p0    Point at the first vertex.
    /// \param[in] _p1    Point at the second vertex.
    /// \param[in] _p2    Point at the third vertex.
    /// \return           The normal vector.
    public: static ignition::math::Vector3d Normal(
      const ignition::math::Vector3d& _v0,
      const ignition::math::Vector3d& _v1,
      const ignition::math::Vector3d& _v2);
  };
}

#endif  // _WAVE_GAZEBO_PLUGINS_GEOMETRY_HH_
