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

#ifndef VRX_GAZEBO_LIGHT_BUOY_PLUGIN_HH_
#define VRX_GAZEBO_LIGHT_BUOY_PLUGIN_HH_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <light_buoy_colors.pb.h>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  typedef const boost::shared_ptr<
    const light_buoy_colors_msgs::msgs::LightBuoyColors>
      ConstLightBuoyColorsPtr;
}

/// \brief Visual plugin for changing the color of some visual elements using
/// ROS messages. This plugin accepts the following SDF parameters:
///
/// <color_1>: The first color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_2>: The second color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_3>: The third color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <shuffle>: True if the topic for shuffling the sequence is enabled.
/// <robot_namespace>: The ROS namespace for this node. If not present,
///                   the model name without any "::"" will be used.
///                   E.g.: The plugin under a visual named
///                   "model1::my_submodel::link::visual" will use "model1"
///                   as namespace unless a value is specified.
/// <ros_shuffle_topic>: The ROS topic used to request color changes.
/// <gz_colors_topic>: The gazebo topic used to request specific color changes.
//    defaults to /vrx/light_buoy/new_pattern
/// <visuals>: The collection of visuals that change in color. It accepts N
///            elements of <visual> elements.
///
/// Here's an example:
///   <plugin name="light_buoy_plugin" filename="liblight_buoy_plugin.so">
///     <color_1>RED</color_1>
///     <color_2>GREEN</color_2>
///     <color_3>BLUE</color_3>
///     <visuals>
///       <visual>robotx_light_buoy::base_link::panel_1</visual>
///       <visual>robotx_light_buoy::base_link::panel_2</visual>
///       <visual>robotx_light_buoy::base_link::panel_3</visual>
///     </visuals>
///     <shuffle>true</shuffle>
///     <robot_namespace>vrx</robot_namespace>
///     <ros_shuffle_topic>light_buoy/shuffle</ros_shuffle_topic>
///   </plugin>
class LightBuoyPlugin : public gazebo::VisualPlugin
{
  // \brief Constructor
  public: LightBuoyPlugin();

  // Documentation inherited.
  public: void Load(gazebo::rendering::VisualPtr _parent,
                    sdf::ElementPtr _sdf);

  /// \brief Creates a std_msgs::ColorRGBA message from 4 doubles.
  /// \param[in] _r Red.
  /// \param[in] _g Green.
  /// \param[in] _b Blue.
  /// \param[in] _a Alpha.
  /// \return The ColorRGBA message.
  private: static std_msgs::ColorRGBA CreateColor(const double _r,
                                                  const double _g,
                                                  const double _b,
                                                  const double _a);

  /// \brief Return the index of the color from its string.
  /// \param[in] _color The color
  /// \return The index in kColors.
  private: static uint8_t IndexFromColor(const std::string &_color);

  /// \brief Initialize all color sequences.
  private: void InitializeAllPatterns();

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  private: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief ROS callback for generating a new color pattern.
  /// \param[in] _msg Not used.
  private: void ChangePattern(const std_msgs::Empty::ConstPtr &_msg);

  /// \brief Gazebo callback for changing light to a specific color pattern.
  /// \param[in] _msg New color sequence.
  private: void ChangePatternTo(gazebo::ConstLightBuoyColorsPtr &_msg);

  /// \brief Display the next color in the sequence, or start over if at the end
  private: void Update();

  /// \def Colors_t
  /// \brief A pair of RGBA color and its name as a string.
  private: using Colors_t = std::pair<std_msgs::ColorRGBA, std::string>;

  /// \def Pattern_t
  /// \brief The current pattern to display, pattern[3] and pattern[4]
  /// are always OFF.
  private: using Pattern_t = std::array<uint8_t, 5>;

  /// \brief List of the color options (red, green, blue, yellow and no color)
  /// with their string name for logging.
  private: static const std::array<Colors_t, 5> kColors;

  /// \brief All color sequences.
  private: std::vector<Pattern_t> allPatterns;

  /// \brief The index pointing to one of the potential color sequences.
  private: size_t allPatternsIdx = 0u;

  /// \brief Collection of visual names.
  private: std::vector<std::string> visualNames;

  /// \brief Pointer to the visual elements to modify.
  private: std::vector<gazebo::rendering::VisualPtr> visuals;

  /// \brief Whether shuffle is enabled via a ROS topic or not.
  private: bool shuffleEnabled = true;

  /// \brief Subscriber to generate and display a new color sequence.
  private: ros::Subscriber changePatternSub;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  // \brief Gazebo Node
  private: gazebo::transport::NodePtr gzNode;

  // \brief Gazebo subscriber listening for color specification
  private: gazebo::transport::SubscriberPtr colorSub;

  /// \brief The color pattern.
  private: Pattern_t pattern;

  /// \brief Track current index in pattern.
  private: uint8_t state = 0u;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic.
  private: std::string rosShuffleTopic;

  /// \brief gazebo topic.
  private: std::string gzColorsTopic;

  /// Pointer to the scene node.
  private: gazebo::rendering::ScenePtr scene;

  /// \brief Connects to rendering update event.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Next time where the plugin should be updated.
  private: gazebo::common::Time nextUpdateTime;

  /// \brief Locks state and pattern member variables.
  private: std::mutex mutex;
};

#endif
