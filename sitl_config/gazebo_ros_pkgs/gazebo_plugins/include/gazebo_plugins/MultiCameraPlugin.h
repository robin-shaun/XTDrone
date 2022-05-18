/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _GAZEBO_MULTI_CAMERA_PLUGIN_HH_
#define _GAZEBO_MULTI_CAMERA_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class MultiCameraPlugin : public SensorPlugin
  {
    public: MultiCameraPlugin();

    /// \brief Destructor
    public: virtual ~MultiCameraPlugin();

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrameLeft(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);
    public: virtual void OnNewFrameRight(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    protected: sensors::MultiCameraSensorPtr parentSensor;

    protected: std::vector<unsigned int> width, height, depth;
    protected: std::vector<std::string> format;

    protected: std::vector<rendering::CameraPtr> camera;

    private: std::vector<event::ConnectionPtr> newFrameConnection;
  };
}
#endif
