/*
 * Copyright (C) 2017  Brian Bingham
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

#ifndef USV_GAZEBO_PLUGINS_WIND_HH_
#define USV_GAZEBO_PLUGINS_WIND_HH_

#include <ros/ros.h>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>

#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A plugin that simulates a simple wind model. It accepts the
  /// following parameters:
  ///
  /// <wind_objs>: Block of objects (models) to be effected by the wind.
  ///   <wind_obj>: A wind object. NOTE: may include as many objects as you like
  ///     <name>: Name of the model (object) that will be effected by the wind
  ///     <link_name>: Link on that model which will feel the force of the wind
  ///                  (limited to ONE per model).
  ///     <coeff_vector>: Coefficient vector of the particluar wind object.
  ///
  /// <wind_direction>: Wind direction vector. Wind direction is specified as
  /// the positive direction of the wind velocity vector in the horizontal plane
  /// in degrees using the ENU coordinate convention
  ///
  /// <wind_mean_velocity>: The wind average velocity.
  ///
  /// <var_wind_gain_constants>: Variable wind speed gain constant.
  ///
  /// <var_wind_time_constants>: Variable wind speed time constant.
  ///
  /// <random_seed>: Set the seed for wind speed randomization.
  ///
  /// <update_rate>: Publishing rate of the wind topic. If set to 0, it will not
  /// publish, if set to a -1 it will publish every simulation iteration.
  /// "Station-keeping control of an unmanned surface vehicle exposed to
  /// current and wind disturbances".
  ///
  /// <topic_wind_speed>: The debug topic to advertise the wind speed.
  ///
  /// <topic_wind_direction>: The debug topic to advertise the wind direction.
  class UsvWindPlugin : public WorldPlugin
  {
    struct WindObj
    {
      /// \Bool to show weather the model and link pointers have been set
      // cppcheck-suppress unusedStructMember
      bool init = false;
      /// \name of model as it will be looked by in the world
      std::string modelName;
      /// \model Pointer to the model
      physics::ModelPtr model;
      /// \Name of the link on that model
      std::string linkName;
      /// \brief Pointer to model link in gazebo,
      ///  optionally specified by the bodyName parameter,
      ///  The states are taken from this link and forces applied to this link.
      physics::LinkPtr link;
      /// \brief Wind force coefficients.
      ignition::math::Vector3d windCoeff;
    };
    /// \brief Constructor.
    public: UsvWindPlugin();

    /// \brief Destructor.
    public: virtual ~UsvWindPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _parent,
                              sdf::ElementPtr _sdf);

    /// \brief Callback executed at every physics update.
    protected: virtual void Update();

    /// \brief vector of simple objects effected by the wind
    private: std::vector<UsvWindPlugin::WindObj> windObjs;

    /// \brief Bool to keep track if ALL of the windObjs have been initialized
    private: bool windObjsInit = false;

    /// \brief Bool to keep track if ALL of the windObjs have been initialized
    private: unsigned int windObjsInitCount = 0;

    /// \brief Pointer to the Gazebo world
    private: physics::WorldPtr world;

    /// \brief Wind velocity unit vector in Gazebo coordinates [m/s].
    private: ignition::math::Vector3d windDirection;

    /// \brief Average wind velocity.
    private: double windMeanVelocity;

    /// \brief User specified gain constant.
    private: double gainConstant;

    /// \brief Calculated filter gain constant.
    private: double filterGain;

    /// \brief Time constant.
    private: double timeConstant;

    /// \brief Previous time.
    private: double previousTime;

    /// \brief Variable velocity component.
    private: double varVel;

    /// \brief ROS node handle.
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Publisher for wind speed.
    private: ros::Publisher windSpeedPub;

    /// \brief Publisher for wind direction.
    private: ros::Publisher windDirectionPub;

    /// \brief Topic where the wind speed is published.
    private: std::string topicWindSpeed = "/vrx/debug/wind/speed";

    /// \brief Topic where the wind direction is published.
    private: std::string topicWindDirection = "/vrx/debug/wind/direction";

    /// \brief Last time wind speed and direction was published.
    private: double lastPublishTime = 0;

    /// \brief Update rate buffer for wind speed and direction.
    private: double updateRate;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \breif Bool debug set by environment var VRX_DEBUG
    private: bool debug = true;

    /// \def Random generator.
    private: std::unique_ptr<std::mt19937> randGenerator;
  };
}

#endif
