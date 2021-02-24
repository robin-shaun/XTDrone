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

#pragma once

#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include "usv_gazebo_plugins/polyhedron_volume.hh"

namespace buoyancy
{
  /// \brief Type of geometry shape
  enum class ShapeType
  {
    None,
    Box,
    Sphere,
    Cylinder
  };

  /// \brief Parent shape object for volume objects
  struct ShapeVolume
  {
    /// \brief Default destructor
    virtual ~ShapeVolume() = default;

    /// \brief Factory method for shape. Parses a shape object from sdf data
    /// \param sdf geometry SDF element
    static std::unique_ptr<ShapeVolume> makeShape(const sdf::ElementPtr sdf);

    /// \brief Display string for shape object
    virtual std::string Display();

    /// \brief Calculates volume + centroid of submerged shape
    /// if the shape is out of water returns Volume{}
    /// @param pose: world pose of volume
    /// @param fluidLevel: height of fluid
    /// @return volume object with volume + centroid (relative to world)
    virtual Volume CalculateVolume(const ignition::math::Pose3d& pose,
                                   double fluidLevel) = 0;

    /// \brief Type of shape
    ShapeType type;

    /// \brief Full volume of object
    double volume;

    /// \brief Average length of object
    /// estimate used for drag torque calculation
    double averageLength;
  };
  typedef std::unique_ptr<ShapeVolume> ShapeVolumePtr;

  /// \brief Box shape volume
  struct BoxVolume : public ShapeVolume
  {
    /// \brief Default constructor
    /// @param x: length
    /// @param y: width
    /// @param z: height
    explicit BoxVolume(double x, double y, double z);

    /// \brief Display string for box shape
    std::string Display() override;

    // Documentation inherited.
    Volume CalculateVolume(const ignition::math::Pose3d& pose,
                           double fluidLevel) override;

    /// \brief Length
    double x;

    /// \brief Width
    double y;

    /// \brief Height
    double z;

    private:
    /// \brief Polyhedron defining a box
    Polyhedron polyhedron;
  };

  /// \brief Cylinder shape volume
  struct CylinderVolume : public ShapeVolume
  {
    /// \brief Default constructor
    /// @param r: radius
    /// @param l: length
    explicit CylinderVolume(double r, double l);

    /// \brief Display string for cylinder shape
    std::string Display() override;

    // Documentation inherited.
    Volume CalculateVolume(const ignition::math::Pose3d& pose,
                           double fluidLevel) override;

    /// \brief Radius of cylinder
    double r;

    /// \brief Height of cylinder
    double h;

    private:
    /// \brief Polyhedron defining a cylinder
    Polyhedron polyhedron;
  };

  /// \brief Sphere shape volume
  struct SphereVolume : public ShapeVolume
  {
    /// \brief Default constructor
    /// @param r: radius
    explicit SphereVolume(double r);

    /// \brief Display string for sphere shape
    std::string Display() override;

    // Documentation inherited.
    Volume CalculateVolume(const ignition::math::Pose3d& pose,
                           double fluidLevel) override;

    /// \brief Radius of sphere
    double r;
  };

  /// \brief Custom exception for parsing errors
  struct ParseException : public std::exception
  {
    ParseException(const char* shape, const char* message)
      : output_("")
    {
      std::stringstream ss;
      ss << "Parse error for <" << shape << ">: " << message;
      // cppcheck-suppress useInitializationList
      this->output_ = ss.str();
    }

    const char* what() const throw()
    {
      return this->output_.c_str();
    }

    private: std::string output_;
  };
}
