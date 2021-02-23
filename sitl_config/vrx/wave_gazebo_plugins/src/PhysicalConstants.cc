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

#include "wave_gazebo_plugins/PhysicalConstants.hh"

namespace asv
{
  /////////////////////////////////////////////////////////////////////////////
  double PhysicalConstants::Gravity()
  {
    return -9.8;
  }

  /////////////////////////////////////////////////////////////////////////////
  double PhysicalConstants::G()
  {
    return 6.67408E-11;
  }

  /////////////////////////////////////////////////////////////////////////////
  double PhysicalConstants::WaterDensity()
  {
    return 998.6;
  }

  /////////////////////////////////////////////////////////////////////////////
  double PhysicalConstants::WaterKinematicViscosity()
  {
    return 1.0533E-6;
  }
}
