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

/// \file Physics.hh
/// \brief This file contains definitions for the physics models.

#ifndef _WAVE_GAZEBO_PLUGINS_PHYSICS_HH_
#define _WAVE_GAZEBO_PLUGINS_PHYSICS_HH_

namespace asv
{

///////////////////////////////////////////////////////////////////////////////
// Physics
  /// \brief A collection of static methods for various physics calculations.
  class Physics
  {
    /// \brief Compute the deep water dispersion.
    ///
    /// \param[in] _wavenumber  The wavenumber: k = 2 PI / wavelength.
    /// \return                 The angular frequency omega.
    public: static double DeepWaterDispersionToOmega(double _wavenumber);

    /// \brief Compute the deep water dispersion.
    ///
    /// \param[in] _omega       The angular frequency: omega = 2 PI / T.
    /// \return                 The wavenumber k.
    public: static double DeepWaterDispersionToWavenumber(double _omega);
  };
}

#endif
