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

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include "gazebo_plugins/gazebo_ros_triggered_multicamera.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTriggeredMultiCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTriggeredMultiCamera::GazeboRosTriggeredMultiCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTriggeredMultiCamera::~GazeboRosTriggeredMultiCamera()
{
}

void GazeboRosTriggeredMultiCamera::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  MultiCameraPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // initialize shared_ptr members
  this->image_connect_count_ = boost::shared_ptr<int>(new int(0));
  this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
  this->was_active_ = boost::shared_ptr<bool>(new bool(false));

  // copying from CameraPlugin into GazeboRosCameraUtils
  for (unsigned i = 0; i < this->camera.size(); ++i)
  {
    GazeboRosTriggeredCamera * cam = new GazeboRosTriggeredCamera();
    cam->parentSensor_ = this->parentSensor;
    cam->width_   = this->width[i];
    cam->height_  = this->height[i];
    cam->depth_   = this->depth[i];
    cam->format_  = this->format[i];
    cam->camera_  = this->camera[i];
    // Set up a shared connection counter
    cam->image_connect_count_ = this->image_connect_count_;
    cam->image_connect_count_lock_ = this->image_connect_count_lock_;
    cam->was_active_ = this->was_active_;
    if (this->camera[i]->Name().find("left") != std::string::npos)
    {
      // FIXME: hardcoded, left hack_baseline_ 0
      cam->Load(_parent, _sdf, "/left", 0.0);
    }
    else if (this->camera[i]->Name().find("right") != std::string::npos)
    {
      double hackBaseline = 0.0;
      if (_sdf->HasElement("hackBaseline"))
        hackBaseline = _sdf->Get<double>("hackBaseline");
      cam->Load(_parent, _sdf, "/right", hackBaseline);
    }
    this->triggered_cameras.push_back(cam);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTriggeredMultiCamera::OnNewFrameLeft(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  GazeboRosTriggeredCamera * cam = this->triggered_cameras[0];
  cam->OnNewFrame(_image, _width, _height, _depth, _format);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTriggeredMultiCamera::OnNewFrameRight(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  GazeboRosTriggeredCamera * cam = this->triggered_cameras[1];
  cam->OnNewFrame(_image, _width, _height, _depth, _format);
}
}
