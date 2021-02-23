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

#ifndef VRX_GAZEBO_NAVIGATION_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_NAVIGATION_SCORING_PLUGIN_HH_

#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/scoring_plugin.hh"

/// \brief A plugin for computing the score of the navigation task.
/// This plugin derives from the generic ScoringPlugin class. Check out that
/// plugin for other required SDF elements.
/// This plugin requires the following SDF parameters:
///
/// <obstacle_penalty>: Specifies how many points are deducted per collision.
/// <gates>: Specifies the collection of gates delimiting the course.
///
///   Each gate accepts the following elements:
///
///   <gate>: A gate is delimited by two markers (left and right).
///   The vessel should pass through the gate with the markers on the defined
///   right and left sides. E.g.:
///
///      <left_marker>: The name of the marker that should stay on the left
///      side of the vessel.
///      <right_marker> The name of the marker that should stay on the right
///      side of the vessel.
///
/// Here's an example:
/// <plugin name="navigation_scoring_plugin"
///         filename="libnavigation_scoring_plugin.so">
///   <vehicle>wamv</vehicle>
///   <task_name>navigation_scoring_plugin</task_name>
///   <course_name>vrx_navigation_course</course_name>
///   <obstacle_penalty>10</obstable_penalty>
///   <gates>
///     <gate>
///       <left_marker>red_bound_0</left_marker>
///       <right_marker>green_bound_0</right_marker>
///     </gate>
///     <gate>
///       <left_marker>red_bound_1</left_marker>
///       <right_marker>green_bound_1</right_marker>
///     </gate>
///     <gate>
///       <left_marker>red_bound_2</left_marker>
///       <right_marker>green_bound_2</right_marker>
///     </gate>
///     <gate>
///       <left_marker>red_bound_3</left_marker>
///       <right_marker>green_bound_3</right_marker>
///     </gate>
///     <gate>
///       <left_marker>red_bound_4</left_marker>
///       <right_marker>green_bound_4</right_marker>
///     </gate>
///     <gate>
///       <left_marker>red_bound_5</left_marker>
///       <right_marker>green_bound_5</right_marker>
///     </gate>
///     <gate>
///       <left_marker>red_bound_6</left_marker>
///       <right_marker>green_bound_6</right_marker>
///     </gate>
///   </gates>
/// </plugin>
class NavigationScoringPlugin : public ScoringPlugin
{
  /// \brief All gate states.
  private: enum class GateState
  {
    /// \brief Not "in" the gate.
    VEHICLE_OUTSIDE,

    /// \brief Before the gate.
    VEHICLE_BEFORE,

    /// \brief After the gate.
    VEHICLE_AFTER,

    /// \brief Gate crossed!
    CROSSED,

    /// \brief Gate invalid. E.g.: if crossed in the wrong direction.
    INVALID,
  };

  /// \brief A gate that is part of the navigation challenge.
  private: class Gate
  {
    /// \brief Constructor.
    /// \param[in] _leftMarkerName The left marker's model.
    /// \param[in] _rightMarkerName The right marker's model.
    public: Gate(const gazebo::physics::LinkPtr _leftMarkerModel,
                 const gazebo::physics::LinkPtr _rightMarkerModel);

    /// \brief Where is the given robot pose with respect to the gate?
    /// \param _robotWorldPose Pose of the robot, in the world frame.
    /// \return The gate state given the current robot pose.
    public: GateState IsPoseInGate(
      const ignition::math::Pose3d &_robotWorldPose) const;

    /// \brief Recalculate the pose and width of the gate.
    public: void Update();

    /// \brief The left marker model.
    public: gazebo::physics::LinkPtr leftMarkerModel;

    /// \brief The right marker model.
    public: gazebo::physics::LinkPtr rightMarkerModel;

    /// \brief The center of the gate in the world frame. Note that the roll and
    /// pitch are ignored. Only yaw is relevant and it points into the direction
    /// in which the gate should be crossed.
    public: ignition::math::Pose3d pose;

    /// \brief The width of the gate in meters.
    public: double width;

    /// \brief The state of this gate.
    public: GateState state = GateState::VEHICLE_OUTSIDE;
  };

  // Constructor.
  public: NavigationScoringPlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::WorldPtr _world,
                    sdf::ElementPtr _sdf);

  /// \brief Parse the gates from SDF.
  /// \param[in] _sdf The current SDF element.
  /// \return True when the gates were successfully parsed or false othwerwise.
  private: bool ParseGates(sdf::ElementPtr _sdf);

  /// \brief Register a new gate.
  /// \param[in] _leftMarkerName The name of the left marker.
  /// \param[in] _rightMarkerName The name of the right marker.
  /// \return True when the gate has been registered or false otherwise.
  private: bool AddGate(const std::string &_leftMarkerName,
                        const std::string &_rightMarkerName);

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief Set the score to 0 and change to state to "finish".
  private: void Fail();

  // Documentation inherited.
  private: void OnCollision() override;

  // Name of Course
  private: gazebo::physics::ModelPtr course;

  /// \brief All the gates.
  private: std::vector<Gate> gates;

  /// \brief Number of gates
  private: int numGates;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief The number of WAM-V collisions.
  private: unsigned int numCollisions = 0;

  /// \brief Number of points deducted per collision.
  private: double obstaclePenalty = 10.0;
};

#endif

