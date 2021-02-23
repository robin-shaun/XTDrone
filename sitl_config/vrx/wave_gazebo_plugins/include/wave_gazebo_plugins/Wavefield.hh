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

/// \file Wavefield.hh
/// \brief This file contains definitions for classes used to manage
/// a wave field. This includes wave parameters, wave generation,
/// and sampling from the wave field.

#ifndef _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_
#define _ASV_WAVE_SIM_GAZEBO_PLUGINS_WAVEFIELD_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <memory>
#include <vector>

namespace asv
{
///////////////////////////////////////////////////////////////////////////////
// WaveParameters
  /// \internal
  /// \brief Class to hold private data for WaveParameters.
  class WaveParametersPrivate;

  /// \brief A class to manage the parameters for generating a wave
    /// in a wave field.
  class WaveParameters
  {
    /// \brief Destructor.
    public: ~WaveParameters();

    /// \brief Constructor.
    public: WaveParameters();

    /// \brief Set the parameters from an SDF Element tree.
    ///
    /// \param[in] _sdf   The SDF Element tree containing the wave parameters.
    public: void SetFromSDF(sdf::Element& _sdf);

    /// \brief The number of wave components (3 max if visualisation required).
    public: size_t Number() const;

    /// \brief The angle between the mean wave direction and the
    ///        largest / smallest component waves.
    public: double Angle() const;

    /// \brief The scale between the mean and largest / smallest
    ///        component waves.
    public: double Scale() const;

    /// \brief A parameter in [0, 1] controlling the wave steepness
    ///        with 1 being steepest.
    public: double Steepness() const;

    /// \brief The angular frequency
    public: double AngularFrequency() const;

    /// \brief The amplitude of the mean wave in [m].
    public: double Amplitude() const;

    /// \brief The period of the mean wave in [s].
    public: double Period() const;

    /// \brief The phase of the mean wave.
    public: double Phase() const;

    /// \brief The mean wavelength.
    public: double Wavelength() const;

    /// \brief The mean wavenumber.
    public: double Wavenumber() const;

        /// \brief Time-constant for starting waves.
    public: float Tau() const;

        /// \brief Amplitude multiplier for PMS.
    public: float Gain() const;

    /// \brief A two component vector specifiying the direction
        /// of the mean wave.
    public: ignition::math::Vector2d Direction() const;

    /// \brief Set the number of wave components (3 max).
    ///
    /// \param[in] _number    The number of component waves.
    public: void SetNumber(size_t _number);

    /// \brief Set the angle parameter controlling
    /// the direction of the component waves.
    ///
    /// \param[in] _angle     The angle parameter.
    public: void SetAngle(double _angle);

    /// \brief Set the scale parameter controlling
    /// the range of amplitudes of the component waves.
    ///
    /// \param[in] _scale   The scale parameter.
    public: void SetScale(double _scale);

    /// \brief Set the steepness parameter controlling
    /// the steepness of the waves. In [0, 1].
    ///
    /// \param[in] _steepness The steepness parameter.
    public: void SetSteepness(double _steepness);

    /// \brief Set the mean wave amplitude. Must be positive.
    ///
    /// \param[in] _amplitude The amplitude parameter.
    public: void SetAmplitude(double _amplitude);

    /// \brief Set the mean wave period. Must be positive.
    ///
    /// \param[in] _period The period parameter.
    public: void SetPeriod(double _period);

    /// \brief Set the mean wave phase.
    ///
    /// \param[in] _phase The phase parameter.
    public: void SetPhase(double _phase);

        /// \brief Set the time constant.
    ///
    /// \param[in] _tau The time constant.
    public: void SetTau(double _tau);

        /// \brief Set the PMS amplitude multiplier
    ///
    /// \param[in] _gain The multiplier
    public: void SetGain(double _gain);

    /// \brief Set the mean wave direction.
    ///
    /// \param[in] _direction The direction parameter, a two component vector.
    public: void SetDirection(const ignition::math::Vector2d& _direction);

    /// \brief Access the component angular frequencies.
    public: const std::vector<double>& AngularFrequency_V() const;

    /// \brief Access the component amplitudes.
    public: const std::vector<double>& Amplitude_V() const;

    /// \brief Access the component phases.
    public: const std::vector<double>& Phase_V() const;

    /// \brief Access the steepness components.
    public: const std::vector<double>& Steepness_V() const;

    /// \brief Access the component wavenumbers.
    public: const std::vector<double>& Wavenumber_V() const;

    /// \brief Access the component directions.
    public: const std::vector<ignition::math::Vector2d>& Direction_V() const;

    /// \brief Print a summary of the wave parameters to the gzmsg stream.
    public: void DebugPrint() const;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<WaveParametersPrivate> data;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldSampler

  /// \brief A class to manage sampling depths from a wave field.
  class WavefieldSampler
  {
    /// \brief Compute the depth at a point directly
        /// (no sampling or interpolation).
    ///
    /// This method solves for (x, y) that when input into the
        /// Gerstner wave function
    /// gives the coordinates of the supplied parameter
        /// _point (_point.x(), _point.y()),
    /// and also computes the wave height pz at this point.
    /// The depth h = pz - point.z().
    /// This is a numerical method that uses a multi-variate
        /// Newton solver to solve
    /// the two dimensional non-linear system. In general it is not as fast as
    /// sampling from a discretised wave field with an efficient
        /// line intersection algorithm.
    ///
    /// \param[in] _waveParams  Gerstner wave parameters.
    /// \param[in] _point       The point at which we want the depth.
    /// \return                 The depth 'h' at the point.
    public: static double ComputeDepthDirectly(
      const WaveParameters& _waveParams,
      const ignition::math::Vector3d& _point,
      double time, double time_init = 0);

    /// A simpler version of determining wave height at a point.
    /// This method enforces that q (steepness) = 0 which allows us
    /// to caculate the wave height exactly for a given 2D point without the
    /// need to interatively solve for the position/height.
    public: static double ComputeDepthSimply(
            const WaveParameters& _waveParams,
      const ignition::math::Vector3d& _point,
            double time, double time_init = 0);
  };
}

#endif
