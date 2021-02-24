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

#ifndef USV_GAZEBO_PLUGINS_DYNAMICS_PLUGIN_HH_
#define USV_GAZEBO_PLUGINS_DYNAMICS_PLUGIN_HH_

#include <Eigen/Core>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/WavefieldModelPlugin.hh"

namespace gazebo
{
  /// \brief Plugin class to implement hydrodynamics and wave response.
  /// This plugin accepts the following SDF parameters:
  ///
  /// <bodyName>: Name of base link for receiving pose and and applying forces.
  /// <boatArea>: Horizontal surface area [m^2]. Default value is 0.48.
  /// <boatLength>: Boat length [m]. Default value is 1.35.
  /// <boatWidth>: Boat width [m]. Default value is 1.
  /// <waterDensity>: Water density [kg/m^3]. Default value is 997.7735.
  /// <waterLevel>: Water height [m]. Default value is 0.5.
  /// <xDotU>: Added mass coeff, surge.
  /// <yDotV>: Added mass coeff, sway.
  /// <zDotW>: Added mass coeff, heave.
  /// <kDotP>: Added mass coeff, roll.
  /// <mDotQ>: Added mass coeff, pitch.
  /// <nDotR>: Added mass coeff, yaw.
  /// <xU>: Linear drag coeff surge.
  /// <xUU>: Quadratic drag coeff surge.
  /// <yV>: Linear drag coeff sway.
  /// <yVV>: Quadratic drag coeff sway
  /// <zW>: Linear drag coeff heave.
  /// <zWW>: Quadratic drag coeff heave.
  /// <kP>: Linear drag coeff roll.
  /// <kPP>: Quadratic drag coeff roll.
  /// <mQ>: Linear drag coeff pitch.
  /// <mQQ>: Quadratic drag coeff pitch.
  /// <nR>: Linear drag coeff yaw.
  /// <nRR>: Quadratic drag coeff yaw.
  /// <wave_n>: Number of waves to generate wave field.
  /// <wave_amp<N>>: Amplitude for each component [m].
  /// <wave_period<N>>: Period for each compenent [s].
  /// <wave_direction<N>>: Direction of motion for each component ENU [rad].
  class UsvDynamicsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: UsvDynamicsPlugin();

    /// \brief Destructor.
    public: virtual ~UsvDynamicsPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Callback for Gazebo simulation engine.
    protected: virtual void Update();

    /// \brief Convenience function for getting SDF parameters.
    /// \param[in] _sdfPtr Pointer to an SDF element to parse.
    /// \param[in] _paramName The name of the element to parse.
    /// \param[in] _defaultVal The default value returned if the element
    /// does not exist.
    /// \return The value parsed.
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                   const std::string &_paramName,
                                   const double _defaultVal) const;

    /// \brief Convenience function for calculating the area of circle segment
    /// \param[in] R Radius of circle
    /// \param[in] h Height of the chord line
    /// \return The area
    private: double CircleSegment(double R, double h);

    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    private: physics::WorldPtr world;

    /// \brief Pointer to model link in gazebo,
    /// optionally specified by the bodyName parameter.
    /// The states are taken from this link and forces applied to this link.
    private: physics::LinkPtr link;

    /// \brief Simulation time of the last update.
    private: common::Time prevUpdateTime;

    /// \brief Linear velocity from previous time step,
    /// for estimating acceleration.
    private: ignition::math::Vector3d prevLinVel;

    /// \brief Angular velocity from previous time step,
    /// for estimating acceleration.
    private: ignition::math::Vector3d prevAngVel;

    /// \brief Values to set via Plugin Parameters.
    /// Plugin Parameter: Added mass in surge, X_\dot{u}.
    private: double paramXdotU;

    /// \brief Plugin Parameter: Added mass in sway, Y_\dot{v}.
    private: double paramYdotV;

    /// \brief Plugin Parameter: Added mass in heave, Z_\dot{w}.
    private: double paramZdotW;

    /// \brief Plugin Parameter: Added mass in roll, K_\dot{p}.
    private: double paramKdotP;

    /// \brief Plugin Parameter: Added mass in pitch, M_\dot{q}.
    private: double paramMdotQ;

    /// \brief Plugin Parameter: Added mass in yaw, N_\dot{r}.
    private: double paramNdotR;

    /// \brief Plugin Parameter: Linear drag in surge.
    private: double paramXu;

    /// \brief Plugin Parameter: Quadratic drag in surge.
    private: double paramXuu;

    /// \brief Plugin Parameter: Linear drag in sway.
    private: double paramYv;

    /// \brief Plugin Parameter: Quadratic drag in sway.
    private: double paramYvv;

    /// \brief Plugin Parameter: Linear drag in heave.
    private: double paramZw;


    /// \brief Plugin Parameter: Quadratic drag in heave.
    private: double paramZww;

    /// \brief Plugin Parameter: Linear drag in roll.
    private: double paramKp;

    /// \brief Plugin Parameter: Quadratic drag in roll.
    private: double paramKpp;

    /// \brief Plugin Parameter: Linear drag in pitch.
    private: double paramMq;

    /// \brief Plugin Parameter: Quadratic drag in pitch.
    private: double paramMqq;

    /// \brief Plugin Parameter: Linear drag in yaw.
    private: double paramNr;

    /// \brief Plugin Parameter: Quadratic drag in yaw.
    private: double paramNrr;

    /// \brief Water height [m].
    private: double waterLevel;

    /// \brief Water density [kg/m^3].
    private: double waterDensity;

    /// \brief Vessel length [m].
    private: double paramBoatLength;

    /// \brief Vessel width [m].
    private: double paramBoatWidth;

    /// \brief Demi-hull radius [m].
    private: double paramHullRadius;

    /// \brief Length discretization, i.e., "N"
    private: int paramLengthN;

    /// \brief Added mass matrix, 6x6.
    private: Eigen::MatrixXd Ma;

    /// \brief The name of the wave model
    protected: std::string waveModelName;

    // /// \brief Wave parameters.
    // private: int paramWaveN;

    // /// \brief Wave amplitude values for N components.
    // private: std::vector<float> paramWaveAmps;

    // /// \brief Wave period values for N components.
    // private: std::vector<float> paramWavePeriods;

    // /// \brief Wave direction values for N components.
    // private: std::vector<std::vector<float>> paramWaveDirections;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief The wave parameters.
    private: std::shared_ptr<const asv::WaveParameters> waveParams;
  };
}

#endif
