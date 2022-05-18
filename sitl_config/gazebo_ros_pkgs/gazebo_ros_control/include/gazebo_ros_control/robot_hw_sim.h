/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/// \brief Plugin template for hardware interfaces for ros_control and Gazebo

/// \author Jonathan Bohren
/// \author Dave Coleman

#ifndef __ROS_CONTROL_GAZEBO_ROBOT_HW_SIM_H
#define __ROS_CONTROL_GAZEBO_ROBOT_HW_SIM_H

#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

namespace gazebo_ros_control {

  // Struct for passing loaded joint data
  struct JointData
  {
    std::string name_;
    std::string hardware_interface_;

    JointData(const std::string& name, const std::string& hardware_interface) :
      name_(name),
      hardware_interface_(hardware_interface)
    {}
  };

  /// \brief Gazebo plugin version of RobotHW
  ///
  /// An object of class RobotHWSim represents a robot's simulated hardware.
  class RobotHWSim : public hardware_interface::RobotHW
  {
  public:

    virtual ~RobotHWSim() { }

    /// \brief Initialize the simulated robot hardware
    ///
    /// Initialize the simulated robot hardware.
    ///
    /// \param robot_namespace  Robot namespace.
    /// \param model_nh  Model node handle.
    /// \param parent_model  Parent model.
    /// \param urdf_model  URDF model.
    /// \param transmissions  Transmissions.
    ///
    /// \return  \c true if the simulated robot hardware is initialized successfully, \c false if not.
    virtual bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions) = 0;

    /// \brief Read state data from the simulated robot hardware
    ///
    /// Read state data, such as joint positions and velocities, from the simulated robot hardware.
    ///
    /// \param time  Simulation time.
    /// \param period  Time since the last simulation step.
    virtual void readSim(ros::Time time, ros::Duration period) = 0;

    /// \brief Write commands to the simulated robot hardware
    ///
    /// Write commands, such as joint position and velocity commands, to the simulated robot hardware.
    ///
    /// \param time  Simulation time.
    /// \param period  Time since the last simulation step.
    virtual void writeSim(ros::Time time, ros::Duration period) = 0;

    /// \brief Set the emergency stop state
    ///
    /// Set the simulated robot's emergency stop state. The default implementation of this function does nothing.
    ///
    /// \param active  \c true if the emergency stop is active, \c false if not.
    virtual void eStopActive(const bool active) {}

  };

}

#endif // ifndef __ROS_CONTROL_GAZEBO_ROBOT_HW_SIM_H
