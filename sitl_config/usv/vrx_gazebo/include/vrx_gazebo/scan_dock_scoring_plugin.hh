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

#ifndef VRX_GAZEBO_SCAN_DOCK_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_SCAN_DOCK_SCORING_PLUGIN_HH_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <light_buoy_colors.pb.h>
#include <dock_placard.pb.h>
#include <memory>
#include <string>
#include <vector>
#include <gazebo/gazebo.hh>
#if GAZEBO_MAJOR_VERSION >= 8
  #include <ignition/transport/Node.hh>
#endif
#include <sdf/sdf.hh>
#include "vrx_gazebo/ColorSequence.h"
#include "vrx_gazebo/scoring_plugin.hh"

/// \brief A class to monitor if the color sequence reported matches the color
/// displayed in the light buoy.
class ColorSequenceChecker
{
  /// \brief Constructor.
  /// \param[in] _expectedColors The sequence of expected colors.
  /// \param[in] _rosNameSpace ROS namespace.
  /// \param[in] _rosColorSequenceService The ROS service used to receive
  /// the color submisison.
  public: ColorSequenceChecker(const std::vector<std::string> &_expectedColors,
                               const std::string &_rosNameSpace,
                               const std::string &_rosColorSequenceService);
  /// \brief Enable the ROS submission service.
  public: void Enable();

  /// \brief Disable the ROS submission service.
  public: void Disable();

  /// \brief Whether a team submitted the color sequence or not.
  /// \return True when the submission was received or false otherwise.
  public: bool SubmissionReceived() const;

  /// \brief Wheter a team submitted the color sequence and is correct or not.
  /// \return True when the team submitted the color sequence and it is correct
  /// or false otherwise.
  public: bool Correct() const;

  /// \brief Callback executed when a new color submission is received.
  /// \param[in] _request Contains the submission.
  /// \param[out] _res The Response. Note that this will be true even if the
  /// reported sequence is incorrect.
  private: bool OnColorSequence(
    ros::ServiceEvent<vrx_gazebo::ColorSequence::Request,
      vrx_gazebo::ColorSequence::Response> &_event);

  /// \brief The expected color sequence.
  private: std::vector<std::string> expectedSequence;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic where the color sequence should be sent.
  private: std::string colorSequenceService;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief Service to generate and display a new color sequence.
  private: ros::ServiceServer colorSequenceServer;

  /// \brief Whether the color sequence has been received or not.
  private: bool colorSequenceReceived = false;

  /// \brief Whether the color sequence received is correct or not.
  private: bool correctSequence = false;
};

/// \brief A class to monitor if the vehicle docked in a given bay.
class DockChecker
{
  /// \brief Constructor.
  /// \param[in] _name The name of the checker (only used for debugging).
  /// \param[in] _internalActivationTopic The gazebo topic used to receive
  /// notifications about the internal activation zone.
  /// \param[in] _externalActivationTopic The gazebo topic used to receive
  /// notifications about the external activation zone.
  /// from the "contain" plugin.
  /// \param[in] _minDockTime Minimum amount of seconds to stay docked to be
  /// considered a fully successfull dock.
  /// \param[in] _dockAllowed Whether is allowed to dock in this bay or not.
  /// \param[in] _worldName Gazebo world name.
  /// \param[in] _announceSymbol Optional symbol to announce via ROS.
  public: DockChecker(const std::string &_name,
                      const std::string &_internalActivationTopic,
                      const std::string &_exteriorActivationTopic,
                      const double _minDockTime,
                      const bool _dockAllowed,
                      const std::string &_worldName,
                      const std::string &_rosNameSpace,
                      const std::string &_announceSymbol,
                      const std::string &_gzSymbolTopic);

  /// \brief The name of this checker.
  public: std::string name;

  /// \brief Whether the robot has been successfully docked in this bay or not.
  /// \return True when the robot has been docked or false otherwise.
  public: bool AnytimeDocked() const;

  /// \brief Whether the robot is currently at the entrance of the bay.
  /// \return True when the robot is at the entrance or false othwerwise.
  public: bool AtEntrance() const;

  /// \brief Whether it is allowed to dock in this bay or not.
  public: bool Allowed() const;

  /// \brief Announce the symbol of the bay via ROS.
  public: void AnnounceSymbol();

  /// \brief Update function that needs to be executed periodically.
  public: void Update();

  /// \brief Callback triggered when the vehicle enters or exits the activation
  /// zone.
  /// \param[in] _msg The current state (0: exiting, 1: entering).
#if GAZEBO_MAJOR_VERSION >= 8
  private: void OnInternalActivationEvent(const ignition::msgs::Boolean &_msg);
#else
  private: void OnInternalActivationEvent(ConstIntPtr &_msg);
#endif

  /// \brief Callback triggered when the vehicle enters or exits the activation
  /// zone.
  /// \param[in] _msg The current state (0: exiting, 1: entering).
#if GAZEBO_MAJOR_VERSION >= 8
  private: void OnExternalActivationEvent(const ignition::msgs::Boolean &_msg);
#else
  private: void OnExternalActivationEvent(ConstIntPtr &_msg);
#endif

  /// \brief The gazebo topic used to receive notifications
  /// from the internal activation zone.
  private: std::string internalActivationTopic;

  /// \brief The gazebo topic used to receive notifications
  /// from the external activation zone.
  private: std::string externalActivationTopic;

  /// \brief The gazebo topic used to publish symbols to the placards
  private: std::string gzSymbolTopic;

  /// \brief Minimum amount of seconds to stay docked to be
  /// considered a fully successfull dock.
  private: double minDockTime;

  /// \brief Whether is allowed to dock in this bay or not.
  private: bool dockAllowed;

  /// \brief Timer used to calculate the elapsed time docked in the bay.
  private: gazebo::common::Timer timer;

#if GAZEBO_MAJOR_VERSION >= 8
  /// \brief Ignition Transport node used for communication.
  private: ignition::transport::Node ignNode;
#endif

  /// \brief Create a node for communication.
  private: gazebo::transport::NodePtr node;

  /// \brief Subscriber to receive notifications from the contain plugin.
  private: gazebo::transport::SubscriberPtr containSub;

  /// \brief Whether the vehicle has successfully docked or not.
  private: bool anytimeDocked = false;

  /// \brief Whether the vehicle is at the entrance of the bay or not.
  private: bool atEntrance = false;

  /// \brief Color and shape of symbol to announce. E.g.: red_cross, blue_circle
  private: std_msgs::String announceSymbol;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS Node handle.
  private: std::unique_ptr<ros::NodeHandle> nh;

  /// \brief Publisher for the symbol.
  private: ros::Publisher symbolPub;

  /// \brief ROS topic where the target symbol will be published.
  private: std::string symbolTopic = "/vrx/scan_dock/placard_symbol";

  /// \brief Publish the placard symbols
  private: gazebo::transport::PublisherPtr dockPlacardPub;
};

/// \brief A plugin for computing the score of the scan and dock task.
/// This plugin derives from the generic ScoringPlugin class. Check out that
/// plugin for other required SDF elements.
/// This plugin requires the following SDF parameters:
///
/// <enable_color_checker>: Optional parameter to turn off color checker
/// service - default is true.
/// <robot_namespace>: Optional parameter with the ROS namespace.
/// <color_sequence_service>: Optional paramter with the ROS service used to
/// receive the color submission.
/// <color_topic>: Optional gazebo topic used to publish the color sequence
///   defaults to /vrx/light_buoy/new_pattern
/// <color_1>: Expected first color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_2>: Expected second color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_3>: Expected third color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_bonus_points>: Points granted when the color sequence is correct.
/// Default value is 10.
/// <bays>: Contains at least one of the following blocks:
///   <bay>: A bay represents a potential play where a vehicle can dock. It has
///   the following required elements:
///     <name>The name of the bay. This is used for debugging only.
///     <internal_activation_topic>The gazebo topic used to receive
///     notifications from the internal activation zone.
///     <external_activation_topic>The gazebo topic used to receive
///     notifications from the external activation zone.
///     <min_dock_time>Minimum amount of seconds to stay docked to be
///     considered a fully successfull dock.
///     <dockAllowed> Whether is allowed to dock in this bay or not.
/// <dock_bonus_points>: Points granted when the vehicle successfully
/// dock-and-undock in any bay.
/// Default value is 10.
/// <correct_dock_bonus_points>: Points granted when the vehicle successfully
/// dock-and-undock in the specified bay.
/// Default value is 10.
/// <symbol>: Required string with format <COLOR>_<SHAPE>, where
/// color can be "red", "green", "blue", "yellow" and color can be "triangle",
/// "circle", "cross". If this parameter is present, a ROS message will be
/// sent in OnReady(). The vehicle should dock in the bay matching this color
/// and shape.
///
/// Here's an example:
/// <plugin name="scan_dock_scoring_plugin"
///               filename="libscan_dock_scoring_plugin.so">
///   <!-- Parameters for scoring_plugin -->
///   <vehicle>wamv</vehicle>
///   <task_name>scan_dock</task_name>
///   <initial_state_duration>3</initial_state_duration>
///   <ready_state_duration>3</ready_state_duration>
///   <running_state_duration>300</running_state_duration>
///   <release_joints>
///     <joint>
///       <name>wamv_external_pivot_joint</name>
///     </joint>
///     <joint>
///       <name>wamv_external_riser</name>
///     </joint>
///   </release_joints>
///
///   <!-- Color sequence checker -->
///   <robot_namespace>vrx</robot_namespace>
///   <color_sequence_service>scan_dock/color_sequence</color_sequence_service>
///   <color_1>red</color_1>
///   <color_2>green</color_2>
///   <color_3>blue</color_3>
///
///   <!-- Dock checkers -->
///   <bays>
///     <bay>
///       <name>bay1</name>
///       <internal_activation_topic>
///         /vrx/dock_2018/bay_1_internal/contain
///       </internal_activation_topic>
///       <external_activation_topic>
///         /vrx/dock_2018/bay_1_external/contain
///       </external_activation_topic>
///       <min_dock_time>10.0</min_dock_time>
///       <dock_allowed>false</dock_allowed>
///     </bay>
///
///     <bay>
///       <name>bay2</name>
///       <internal_activation_topic>
///         /vrx/dock_2018/bay_2_internal/contain
///       </internal_activation_topic>
///       <external_activation_topic>
///         /vrx/dock_2018/bay_2_external/contain
///       </external_activation_topic>
///       <min_dock_time>10.0</min_dock_time>
///       <dock_allowed>true</dock_allowed>
///       <symbol>red_circle</symbol>
///     </bay>
///   </bays>
/// </plugin>
class ScanDockScoringPlugin : public ScoringPlugin
{
  // Documentation inherited.
  public: ScanDockScoringPlugin();

  // Documentation inherited.
  private: void Load(gazebo::physics::WorldPtr _world,
                     sdf::ElementPtr _sdf);

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  private: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief Callback executed at every world update.
  private: void Update();

  // Documentation inherited.
  private: void OnReady() override;

  // Documentation inherited.
  private: void OnRunning() override;

  /// \brief gazebo Node
  private: gazebo::transport::NodePtr node;

  /// \brief Publish the color sequence
  private: gazebo::transport::PublisherPtr lightBuoySequencePub;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief In charge of receiving the team submission and compare it with
  /// the color sequence from the light buoy.
  private: std::unique_ptr<ColorSequenceChecker> colorChecker;

  /// \brief Monitor all the available bays to decide when the vehicle docks.
  private: std::vector<std::unique_ptr<DockChecker>> dockCheckers;

  /// \brief To check colors or not
  private: bool enableColorChecker = true;

  /// \brief Whether we have processed the color sequence submission or not.
  private: bool colorSubmissionProcessed = false;

  /// \brief Points granted when the color sequence is correct.
  private: double colorBonusPoints = 10.0;

  /// \brief Points granted when the vehicle successfully
  /// dock-and-undock in any bay
  private: double dockBonusPoints = 10.0;

  /// \brief Points granted when the vehicle successfully
  /// dock-and-undock in the specified bay.
  private: double correctDockBonusPoints = 10.0;

  /// \brief Name of colorTopic for the light buoy
  private: std::string colorTopic;

  /// \brief Expected color sequence.
  private: std::vector<std::string> expectedSequence;
};

#endif
