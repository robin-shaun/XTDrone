/*
 * Copyright 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_ROS_TRIGGERED_CAMERA_HH
#define GAZEBO_ROS_TRIGGERED_CAMERA_HH

#include <mutex>
#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
  class GazeboRosTriggeredMultiCamera;
  class GazeboRosTriggeredCamera : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosTriggeredCamera();

    /// \brief Destructor
    public: ~GazeboRosTriggeredCamera();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Load the plugin.
    /// \param[in] _parent Take in SDF root element.
    /// \param[in] _sdf SDF values.
    /// \param[in] _camera_name_suffix Suffix of the camera name.
    /// \param[in] _hack_baseline Multiple camera baseline.
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf,
                      const std::string &_camera_name_suffix,
                      double _hack_baseline);

    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    protected: virtual void TriggerCamera();

    protected: virtual bool CanTriggerCamera();

    protected: event::ConnectionPtr preRenderConnection_;

    public: void SetCameraEnabled(const bool _enabled);

    protected: void PreRender();

    protected: int triggered = 0;

    protected: std::mutex mutex;

    friend class GazeboRosTriggeredMultiCamera;
  };
}
#endif

