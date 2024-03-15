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

#include "gazebo_plugins/gazebo_ros_triggered_camera.h"

#include <float.h>
#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTriggeredCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTriggeredCamera::GazeboRosTriggeredCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTriggeredCamera::~GazeboRosTriggeredCamera()
{
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
}

void GazeboRosTriggeredCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosTriggeredCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  GazeboRosCameraUtils::Load(_parent, _sdf);

  this->SetCameraEnabled(false);
  this->preRenderConnection_ =
      event::Events::ConnectPreRender(
          std::bind(&GazeboRosTriggeredCamera::PreRender, this));
}

void GazeboRosTriggeredCamera::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix,
  double _hack_baseline)
{
  GazeboRosCameraUtils::Load(_parent, _sdf, _camera_name_suffix, _hack_baseline);

  this->SetCameraEnabled(false);
  this->preRenderConnection_ =
      event::Events::ConnectPreRender(
      std::bind(&GazeboRosTriggeredCamera::PreRender, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTriggeredCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosTriggeredCamera::OnNewFrame");
#endif
  this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();

  if ((*this->image_connect_count_) > 0)
  {
#ifdef ENABLE_PROFILER
    IGN_PROFILE_BEGIN("PutCameraData");
#endif
    this->PutCameraData(_image);
#ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishCameraInfo");
#endif
    this->PublishCameraInfo();
#ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
#endif
  }
#ifdef ENABLE_PROFILER
  IGN_PROFILE_BEGIN("SetCameraEnabled");
#endif
  this->SetCameraEnabled(false);
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
  std::lock_guard<std::mutex> lock(this->mutex);
  this->triggered = std::max(this->triggered-1, 0);
}

void GazeboRosTriggeredCamera::TriggerCamera()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->parentSensor_)
    return;
  this->triggered++;
}

bool GazeboRosTriggeredCamera::CanTriggerCamera()
{
  return true;
}

void GazeboRosTriggeredCamera::PreRender()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->triggered > 0)
  {
    this->SetCameraEnabled(true);
  }
}

void GazeboRosTriggeredCamera::SetCameraEnabled(const bool _enabled)
{
  this->parentSensor_->SetActive(_enabled);
  this->parentSensor_->SetUpdateRate(_enabled ? 0.0 : DBL_MIN);
}

}
