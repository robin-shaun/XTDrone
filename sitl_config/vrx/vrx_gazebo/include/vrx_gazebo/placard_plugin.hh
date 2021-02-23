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

#ifndef VRX_GAZEBO_PLACARD_PLUGIN_HH_
#define VRX_GAZEBO_PLACARD_PLUGIN_HH_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <dock_placard.pb.h>
#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  typedef const boost::shared_ptr<
    const dock_placard_msgs::msgs::DockPlacard>
      ConstDockPlacardPtr;
}

/// \brief Controls the shape and color of a symbol.
///
/// It selects a shape (triangle, cross, circle) and a color (red, green, blue)
/// and applies to the current symbol
/// This plugin can be configured with the following SDF tags:
///
///   <shape> "triangle", "cross" or "circle". If ommited, the shape will be
///           randomly selected.
///   <color> "red", "green" or "blue". If ommited, the color will be
///           randomly selected.
///   <shuffle>: True if the topic for shuffling the sequence is enabled.
///   <robot_namespace> ROS namespace of Node, can be used to have multiple
///                    plugins.
///   <ros_shuffle_topic>: The ROS topic used to request color changes.
///   <gz_symbol_topic>: The gazebo topic subscribed to set symbol changes
///     defaults to /<robot_namespace>/symbol
///   <visuals>: The set of visual symbols. It contains at least one visual:
///     <visual>: A visual displaying a shape.
///
/// Here's an example:
///   <plugin name="placard1plugin" filename="libplacard_plugin.so">
///     <shape>triangle</shape>
///     <color>red</color>
///     <visuals>
///       <visual>dock_2018_placard1::visual_circle</visual>
///       <visual>dock_2018_placard1::visual_h_cross</visual>
///       <visual>dock_2018_placard1::visual_v_cross</visual>
///       <visual>dock_2018_placard1::visual_triangle</visual>
///     </visuals>
///     <shuffle>true</shuffle>
///     <robot_namespace>vrx</robot_namespace>
///     <topic>dock/placard/shuffle</topic>
///   </plugin>
class PlacardPlugin : public gazebo::VisualPlugin
{
  // Documentation inherited.
  public: PlacardPlugin();

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

  /// \brief Initialize all color/symbol sequences.
  private: void InitializeAllPatterns();

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  private: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief Display the symbol in the placard
  private: void Update();

  /// \brief ROS callback for changing a symbol and its color.
  /// \param[in] _msg Not used.
  private: void ChangeSymbol(const std_msgs::Empty::ConstPtr &_msg);

  /// \brief Gazebo callback for changing light to a specific color pattern.
  /// \param[in] _msg New symbol.
  private: void ChangeSymbolTo(gazebo::ConstDockPlacardPtr &_msg);

  /// \brief List of the color options (red, green, blue, and no color)
  /// with their string name for logging.
  private: static std::map<std::string, std_msgs::ColorRGBA> kColors;

  /// \brief List of the shape options (circle, cross, triangle)
  /// with their string name for logging.
  private: static std::vector<std::string> kShapes;

  /// \brief The current color.
  private: std::string color;

  /// \brief The current shape.
  private: std::string shape;

  /// \brief All color/symbol sequences.
  private: std::vector<std::array<std::string, 2u>> allPatterns;

  /// \brief The index pointing to one of the potential color/symbol sequence.
  private: size_t allPatternsIdx = 0u;

  /// \brief Collection of visual names.
  private: std::vector<std::string> visualNames;

  /// \brief Pointer to the visual elements to modify.
  private: std::vector<gazebo::rendering::VisualPtr> visuals;

  /// \brief Whether shuffle is enabled via a ROS topic or not.
  private: bool shuffleEnabled = true;

  /// \brief Service to generate and display a new symbol.
  private: ros::Subscriber changeSymbolSub;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic.
  private: std::string rosShuffleTopic;

  /// \brief gazebo Node
  private: gazebo::transport::NodePtr gzNode;

  /// \brief gazebo symbol sub topic
  private: std::string symbolSubTopic;

  /// \breif symbol subscriber
  private: gazebo::transport::SubscriberPtr symbolSub;

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
