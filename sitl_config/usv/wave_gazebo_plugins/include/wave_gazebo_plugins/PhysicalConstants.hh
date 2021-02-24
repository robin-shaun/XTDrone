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

/// \file PhysicalConstants.hh
/// \brief This file contains definitions of some physical constants used
/// in the physics calculations.

#ifndef _WAVE_GAZEBO_PLUGINS_PHYSICAL_CONSTANTS_HH_
#define _WAVE_GAZEBO_PLUGINS_PHYSICAL_CONSTANTS_HH_

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// PhysicalConstants

  /// \brief A collection of static methods to retrieve physical constants.
  class PhysicalConstants
  {
    /// \brief Uniform acceleration due to gravity
    /// at earth's surface (orientation is z-up).
    ///
    /// \return     -9.8 [m s-2].
    public: static double Gravity();

    /// \brief Universal gravitational constant.
    ///
    /// \return     6.67408E-11 [m3 kg-1 s-2].
    public: static double G();

    /// \brief Density of water.
    ///
    /// \return     998.6 [kg m-3].
    public: static double WaterDensity();

    /// \brief Kinematic viscosity of water at 18 dgree C.
    ///
    /// Source:
    /// <https://www.engineeringtoolbox.com/water-dynamic-kinematic-viscosity-d_596.html>
    ///
    /// \return     1.0533E-6 [m2 s-1].
    public: static double WaterKinematicViscosity();
  };
}

#endif
