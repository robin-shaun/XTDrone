/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "usv_gazebo_plugins/shape_volume.hh"

using namespace buoyancy;

/////////////////////////////////////////////////
ShapeVolumePtr ShapeVolume::makeShape(const sdf::ElementPtr sdf)
{
  double epsilon = 1e-20;

  ShapeVolume* shape = nullptr;

  if (sdf->HasElement("box"))
  {
    auto boxElem = sdf->GetElement("box");
    if (boxElem->HasElement("size"))
    {
      ignition::math::Vector3d dim = boxElem->GetElement("size")
          ->Get<ignition::math::Vector3d>();
      if (dim[0] > epsilon && dim[1] > epsilon && dim[2] > epsilon)
      {
        shape = dynamic_cast<ShapeVolume*>(
            new BoxVolume(dim[0], dim[1], dim[2]));
      }
      else
      {
        throw ParseException("box", "incorrect dimensions");
      }
    }
    else
    {
      throw ParseException("box", "missing <size> element");
    }
  }
  else if (sdf->HasElement("sphere"))
  {
    auto sphereElem = sdf->GetElement("sphere");
    if (sphereElem->HasElement("radius"))
    {
      auto r = sphereElem->GetElement("radius")->Get<double>();
      if (r > epsilon)
      {
        shape = dynamic_cast<ShapeVolume*>(new SphereVolume(r));
      }
      else
      {
        throw ParseException("sphere", "incorrect dimensions");
      }
    }
    else
    {
      throw ParseException("sphere", "missing <radius> element");
    }
  }
  else if (sdf->HasElement("cylinder"))
  {
    auto cylinderElem = sdf->GetElement("cylinder");
    if (cylinderElem->HasElement("radius") &&
        cylinderElem->HasElement("length"))
    {
      auto r = cylinderElem->GetElement("radius")->Get<double>();
      auto l = cylinderElem->GetElement("length")->Get<double>();
      if (r > epsilon || l > epsilon)
      {
        shape = dynamic_cast<ShapeVolume*>(new CylinderVolume(r, l));
      }
      else
      {
        throw ParseException("cylinder", "incorrect dimensions");
      }
    }
    else
    {
      throw ParseException("cylinder", "missing <radius> or <length> element");
    }
  }
  else
  {
    throw ParseException(
        "geometry", "missing <box>, <cylinder> or <sphere> element");
  }

  return std::unique_ptr<ShapeVolume>(shape);
}

/////////////////////////////////////////////////
std::string ShapeVolume::Display()
{
  switch (type)
  {
    case ShapeType::None:
      return "None";
    case ShapeType::Box:
      return "Box";
    case ShapeType::Cylinder:
      return "Cylinder";
    case ShapeType::Sphere:
      return "Sphere";
  }
}

//////////////////////////////////////////////////
BoxVolume::BoxVolume(double x, double y, double z)
    : x(x),
      y(y),
      z(z),
      polyhedron(Polyhedron::makeCube(x, y, z))
{
  type = ShapeType::Box;
  volume = x * y * z;
  averageLength = (x + y + z) / 3.0;
}

//////////////////////////////////////////////////
std::string BoxVolume::Display()
{
  std::stringstream ss;
  ss << ShapeVolume::Display() << ":" << x << "," << y << "," << z;
  return ss.str();
}

//////////////////////////////////////////////////
Volume BoxVolume::CalculateVolume(const ignition::math::Pose3d &pose,
                                  double fluidLevel)
{
  Plane waterSurface;
  waterSurface.offset = fluidLevel;
  return this->polyhedron.SubmergedVolume(pose.Pos(), pose.Rot(), waterSurface);
}

/////////////////////////////////////////////////
CylinderVolume::CylinderVolume(double r, double h)
    : r(r),
      h(h),
      polyhedron(Polyhedron::makeCylinder(r, h, 20))
{
  type = ShapeType::Cylinder;
  volume = M_PI * r * r * h;
  averageLength = (2 * r + h) / 2.0;
}

/////////////////////////////////////////////////
std::string CylinderVolume::Display()
{
  std::stringstream ss;
  ss << ShapeVolume::Display() << ":" << r << "," << h;
  return ss.str();
}

/////////////////////////////////////////////////
Volume CylinderVolume::CalculateVolume(const ignition::math::Pose3d &pose,
                                       double fluidLevel)
{
  Plane waterSurface;
  waterSurface.offset = fluidLevel;
  return this->polyhedron.SubmergedVolume(pose.Pos(), pose.Rot(), waterSurface);
}

//////////////////////////////////////////////////
SphereVolume::SphereVolume(double r)
    : r(r)
{
  type = ShapeType::Sphere;
  volume = 4./3. * M_PI * r * r * r;
  averageLength = 2 * r;
}

//////////////////////////////////////////////////
std::string SphereVolume::Display()
{
  std::stringstream ss;
  ss << ShapeVolume::Display() << ":" << r;
  return ss.str();
}

//////////////////////////////////////////////////
Volume SphereVolume::CalculateVolume(const ignition::math::Pose3d &pose,
                                     double fluidLevel)
{
  Volume output{};
  // Location of bottom of object relative to the fluid surface - assumes
  // origin is at cog of the object.
  double h = fluidLevel - (pose.Pos().Z() - r);

  if (h <= 0) {
    return output;
  }

  // limits of integration
  double intLimitLower = -r;
  double intLimitUpper = (-r + h) > r ? r : (-r + h);

  // volume = integral of (R^2 - z^2) dz * pi
  output.volume = (pow(r, 2) * (intLimitUpper - intLimitLower)
      - (pow(intLimitUpper, 3)/3.0 - pow(intLimitLower, 3)/3.0)) * M_PI;
  output.centroid.X() = pose.Pos().X();
  output.centroid.Y() = pose.Pos().Y();

  if (output.volume > 0) {
    // centroid is always centered to object in X and Y plane
    output.centroid.X() = pose.Pos().X();
    output.centroid.Y() = pose.Pos().Y();
    // z_bar = (integral of (z(R)^2 - z^3) dz) * pi / volume
    output.centroid.Z() = (pow(r, 2) / 2.0 *
        (pow(intLimitUpper, 2) - pow(intLimitLower, 2)) -
        (pow(intLimitUpper, 4) - pow(intLimitLower, 4))/ 4.0)
            * M_PI / output.volume;
    // convert centroid.z to global frame
    output.centroid.Z() = pose.Pos().Z() + output.centroid.Z();
  }
  return output;
}
