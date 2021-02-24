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

#include <cmath>

#include <ignition/math/config.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "wave_gazebo_plugins/Geometry.hh"

/* The version of ignition math in Ubuntu Xenial is 2.2.3 and lacks of
 * some features added after that version in the 2.x series */

/* There is a bug in versions for ign-math in Bionic that does not
 * define properly IGNITION_MATH_MAJOR_VERSION. Some magic to check
 * defined variables but empty and assume 3.0.x series */
#define DO_EXPAND(VAL)  VAL ## 1
#define EXPAND(VAL)     DO_EXPAND(VAL)
#if (EXPAND(IGNITION_MATH_MAJOR_VERSION) == 1)
  #define MAJOR_VERSION 3
  #define MINOR_VERSION 0
#else
  #define MAJOR_VERSION IGNITION_MATH_MAJOR_VERSION
  #define MINOR_VERSION IGNITION_MATH_MINOR_VERSION
#endif

#if MAJOR_VERSION == 2 && MINOR_VERSION < 3
    #define ign_math_vector2d_zero ignition::math::Vector2d(0, 0)
    #define ign_math_vector3d_zero ignition::math::Vector3d(0, 0, 0)
    #define _v_length sqrt(std::pow(_v[0], 2) + std::pow(_v[1], 2))
    #define n_length sqrt(std::pow(n[0], 2) + std::pow(n[1], 2))
#else
  #define ign_math_vector2d_zero ignition::math::Vector2d::Zero
  #define ign_math_vector3d_zero ignition::math::Vector3d::Zero
  #define _v_length _v.Length()
  #define n_length n.Length()
#endif

namespace asv
{
  /////////////////////////////////////////////////
  ignition::math::Vector2d
  Geometry::Normalize(const ignition::math::Vector2d& _v)
  {
    if (_v == ign_math_vector2d_zero)
      return _v;
    else
      return _v/_v_length;
  }

  /////////////////////////////////////////////////
  ignition::math::Vector3d
  Geometry::Normalize(const ignition::math::Vector3d& _v)
  {
    if (_v == ign_math_vector3d_zero)
      return _v;
    else
      return _v/_v_length;
  }

  /////////////////////////////////////////////////
  ignition::math::Vector3d Geometry::Normal(
    const ignition::math::Vector3d& _p0,
    const ignition::math::Vector3d& _p1,
    const ignition::math::Vector3d& _p2
  )
  {
    auto n = ignition::math::Vector3d::Normal(_p0, _p1, _p2);
    if (n == ign_math_vector3d_zero)
      return n;
    else
      return n/n_length;
  }
}
