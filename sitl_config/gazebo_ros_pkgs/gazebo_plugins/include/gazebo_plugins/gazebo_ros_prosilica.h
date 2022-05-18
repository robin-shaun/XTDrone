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

#ifndef GAZEBO_ROS_PROSILICA_CAMERA_HH
#define GAZEBO_ROS_PROSILICA_CAMERA_HH

#include <boost/thread/mutex.hpp>

// library for processing camera data for gazebo / ros conversions
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo/plugins/CameraPlugin.hh>

// ros
#include <ros/callback_queue.h>

// image components
#include <cv_bridge/cv_bridge.h>
// used by polled_camera
#include <sensor_msgs/RegionOfInterest.h>

// prosilica components
// Stuff in image_common
#include <image_transport/image_transport.h>
#include <polled_camera/publication_server.h>  // do: sudo apt-get install ros-hydro-polled-camera
#include <polled_camera/GetPolledImage.h>

namespace gazebo
{

class GazeboRosProsilica : public CameraPlugin, GazeboRosCameraUtils
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosProsilica();

  /// \brief Destructor
  public: virtual ~GazeboRosProsilica();

  /// \brief Load the controller
  /// \param node XML config node
  public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// \brief does nothing for now
  private: static void mouse_cb(int event, int x, int y, int flags, void* param) { };

  /// \brief image_transport
  private: polled_camera::PublicationServer poll_srv_;      // Handles requests in polled mode

  private: std::string mode_;

  private: std::string mode_param_name;
/*
  /// \brief Service call to publish images, cam info
  private: bool camInfoService(prosilica_camera::CameraInfo::Request &req,
                               prosilica_camera::CameraInfo::Response &res);
  private: bool triggeredGrab(prosilica_camera::PolledImage::Request &req,
                              prosilica_camera::PolledImage::Response &res);
*/

  private: void pollCallback(polled_camera::GetPolledImage::Request& req,
                             polled_camera::GetPolledImage::Response& rsp,
                             sensor_msgs::Image& image, sensor_msgs::CameraInfo& info);

  /// \brief ros message
  /// \brief construct raw stereo message
  private: sensor_msgs::Image *roiImageMsg;
  private: sensor_msgs::CameraInfo *roiCameraInfoMsg;

  /// \brief ROS image topic name
  private: std::string pollServiceName;

  private: void Advertise();
  private: event::ConnectionPtr load_connection_;

  // subscribe to world stats
  //private: transport::NodePtr node;
  //private: transport::SubscriberPtr statsSub;
  //private: common::Time simTime;
  //public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);

  /// \brief Update the controller
  protected: virtual void OnNewImageFrame(const unsigned char *_image,
                 unsigned int _width, unsigned int _height,
                 unsigned int _depth, const std::string &_format);

};

}
#endif

