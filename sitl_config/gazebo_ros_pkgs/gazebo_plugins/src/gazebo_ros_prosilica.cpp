/*
 * Copyright 2013 Open Source Robotics Foundation
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

/*
 @mainpage
   Desc: GazeboRosProsilica plugin for simulating Prosilica cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_prosilica.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/rendering/Camera.hh>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <opencv2/highgui.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <string>
#include <chrono>
#include <thread>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosProsilica::GazeboRosProsilica()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosProsilica::~GazeboRosProsilica()
{
  // Finalize the controller
  this->poll_srv_.shutdown();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosProsilica::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{

  CameraPlugin::Load(_parent, _sdf);
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;
  GazeboRosCameraUtils::Load(_parent, _sdf);

  this->load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboRosProsilica::Advertise, this));
}

void GazeboRosProsilica::Advertise()
{
  // camera mode for prosilica:
  // prosilica::AcquisitionMode mode_; /// @todo Make this property of Camera

  //ROS_ERROR_NAMED("prosilica", "before trigger_mode %s %s",this->mode_param_name.c_str(),this->mode_.c_str());

  if (!this->rosnode_->searchParam("trigger_mode",this->mode_param_name)) ///\@todo: hardcoded per prosilica_camera wiki api, make this an urdf parameter
      this->mode_param_name = "trigger_mode";

  if (!this->rosnode_->getParam(this->mode_param_name,this->mode_))
      this->mode_ = "streaming";

  ROS_INFO_NAMED("prosilica", "trigger_mode %s %s",this->mode_param_name.c_str(),this->mode_.c_str());


  if (this->mode_ == "polled")
  {
      poll_srv_ = polled_camera::advertise(*this->rosnode_,this->pollServiceName,&GazeboRosProsilica::pollCallback,this);
  }
  else if (this->mode_ == "streaming")
  {
      ROS_DEBUG_NAMED("prosilica", "do nothing here,mode: %s",this->mode_.c_str());
  }
  else
  {
      ROS_ERROR_NAMED("prosilica", "trigger_mode is invalid: %s, using streaming mode",this->mode_.c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosProsilica::OnNewImageFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosProsilica::OnNewImageFrame");
#endif
  if (!this->rosnode_->getParam(this->mode_param_name,this->mode_))
      this->mode_ = "streaming";

  // should do nothing except turning camera on/off, as we are using service.
  /// @todo: consider adding thumbnailing feature here if subscribed.
  common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();

  // as long as ros is connected, parent is active
  //ROS_ERROR_NAMED("prosilica", "debug image count %d",this->image_connect_count_);
  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    //ROS_ERROR_NAMED("prosilica", "camera_ new frame %s %s",this->parentSensor_->Name().c_str(),this->frame_name_.c_str());

    if (this->mode_ == "streaming")
    {
      if ((*this->image_connect_count_) > 0)
      {
        if (sensor_update_time - this->last_update_time_ >= this->update_period_)
        {
#ifdef ENABLE_PROFILER
          IGN_PROFILE_BEGIN("PutCameraData");
#endif
          this->PutCameraData(_image, sensor_update_time);
#ifdef ENABLE_PROFILER
          IGN_PROFILE_END();
          IGN_PROFILE_BEGIN("PublishCameraInfo");
#endif
          this->PublishCameraInfo(sensor_update_time);
#ifdef ENABLE_PROFILER
          IGN_PROFILE_END();
#endif
          this->last_update_time_ = sensor_update_time;
        }
      }
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
// new prosilica interface.
void GazeboRosProsilica::pollCallback(polled_camera::GetPolledImage::Request& req,
                                      polled_camera::GetPolledImage::Response& rsp,
                                      sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
{
  if (!this->rosnode_->getParam(this->mode_param_name,this->mode_))
      this->mode_ = "streaming";

  /// @todo Support binning (maybe just cv::resize)
  /// @todo Don't adjust K, P for ROI, set CameraInfo.roi fields instead
  /// @todo D parameter order is k1, k2, t1, t2, k3

  if (this->mode_ != "polled")
  {
    rsp.success = false;
    rsp.status_message = "Camera is not in triggered mode";
    return;
  }

  if (req.binning_x > 1 || req.binning_y > 1)
  {
    rsp.success = false;
    rsp.status_message = "Gazebo Prosilica plugin does not support binning";
    return;
  }

  // get region from request
  if (req.roi.x_offset <= 0 || req.roi.y_offset <= 0 || req.roi.width <= 0 || req.roi.height <= 0)
  {
    req.roi.x_offset = 0;
    req.roi.y_offset = 0;
    req.roi.width = this->width_;
    req.roi.height = this->height;
  }
  const unsigned char *src = NULL;
  ROS_ERROR_NAMED("prosilica", "roidebug %d %d %d %d", req.roi.x_offset, req.roi.y_offset, req.roi.width, req.roi.height);

  // signal sensor to start update
  (*this->image_connect_count_)++;

  // wait until an image has been returned
  while(!src)
  {
    {
      // Get a pointer to image data
      src = this->parentSensor->Camera()->ImageData(0);

      if (src)
      {

        // fill CameraInfo
        this->roiCameraInfoMsg = &info;
        this->roiCameraInfoMsg->header.frame_id = this->frame_name_;

        common::Time roiLastRenderTime = this->parentSensor_->LastMeasurementTime();
        this->roiCameraInfoMsg->header.stamp.sec = roiLastRenderTime.sec;
        this->roiCameraInfoMsg->header.stamp.nsec = roiLastRenderTime.nsec;

        this->roiCameraInfoMsg->width  = req.roi.width; //this->parentSensor->ImageWidth() ;
        this->roiCameraInfoMsg->height = req.roi.height; //this->parentSensor->ImageHeight();
        // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
        this->roiCameraInfoMsg->distortion_model = "plumb_bob";
        this->roiCameraInfoMsg->D.resize(5);
#endif
        this->roiCameraInfoMsg->D[0] = this->distortion_k1_;
        this->roiCameraInfoMsg->D[1] = this->distortion_k2_;
        this->roiCameraInfoMsg->D[2] = this->distortion_k3_;
        this->roiCameraInfoMsg->D[3] = this->distortion_t1_;
        this->roiCameraInfoMsg->D[4] = this->distortion_t2_;
        // original camera matrix
        this->roiCameraInfoMsg->K[0] = this->focal_length_;
        this->roiCameraInfoMsg->K[1] = 0.0;
        this->roiCameraInfoMsg->K[2] = this->cx_ - req.roi.x_offset;
        this->roiCameraInfoMsg->K[3] = 0.0;
        this->roiCameraInfoMsg->K[4] = this->focal_length_;
        this->roiCameraInfoMsg->K[5] = this->cy_ - req.roi.y_offset;
        this->roiCameraInfoMsg->K[6] = 0.0;
        this->roiCameraInfoMsg->K[7] = 0.0;
        this->roiCameraInfoMsg->K[8] = 1.0;
        // rectification
        this->roiCameraInfoMsg->R[0] = 1.0;
        this->roiCameraInfoMsg->R[1] = 0.0;
        this->roiCameraInfoMsg->R[2] = 0.0;
        this->roiCameraInfoMsg->R[3] = 0.0;
        this->roiCameraInfoMsg->R[4] = 1.0;
        this->roiCameraInfoMsg->R[5] = 0.0;
        this->roiCameraInfoMsg->R[6] = 0.0;
        this->roiCameraInfoMsg->R[7] = 0.0;
        this->roiCameraInfoMsg->R[8] = 1.0;
        // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
        this->roiCameraInfoMsg->P[0] = this->focal_length_;
        this->roiCameraInfoMsg->P[1] = 0.0;
        this->roiCameraInfoMsg->P[2] = this->cx_ - req.roi.x_offset;
        this->roiCameraInfoMsg->P[3] = -this->focal_length_ * this->hack_baseline_;
        this->roiCameraInfoMsg->P[4] = 0.0;
        this->roiCameraInfoMsg->P[5] = this->focal_length_;
        this->roiCameraInfoMsg->P[6] = this->cy_ - req.roi.y_offset;
        this->roiCameraInfoMsg->P[7] = 0.0;
        this->roiCameraInfoMsg->P[8] = 0.0;
        this->roiCameraInfoMsg->P[9] = 0.0;
        this->roiCameraInfoMsg->P[10] = 1.0;
        this->roiCameraInfoMsg->P[11] = 0.0;
        this->camera_info_pub_.publish(*this->roiCameraInfoMsg);

        // copy data into image_msg_, then convert to roiImageMsg(image)
        this->image_msg_.header.frame_id    = this->frame_name_;

        common::Time lastRenderTime = this->parentSensor_->LastMeasurementTime();
        this->image_msg_.header.stamp.sec = lastRenderTime.sec;
        this->image_msg_.header.stamp.nsec = lastRenderTime.nsec;

        //unsigned char dst[this->width_*this->height];

        /// @todo: don't bother if there are no subscribers

        // copy from src to image_msg_
        fillImage(this->image_msg_,
                  this->type_,
                  this->height_,
                  this->width_,
                  this->skip_*this->width_,
                  (void*)src );

        /// @todo: publish to ros, thumbnails and rect image in the Update call?

        this->image_pub_.publish(this->image_msg_);

        {
          // copy data into ROI image
          this->roiImageMsg = &image;
          this->roiImageMsg->header.frame_id = this->frame_name_;
          common::Time roiLastRenderTime = this->parentSensor_->LastMeasurementTime();
          this->roiImageMsg->header.stamp.sec = roiLastRenderTime.sec;
          this->roiImageMsg->header.stamp.nsec = roiLastRenderTime.nsec;

          // convert image_msg_ to a CvImage using cv_bridge
          boost::shared_ptr<cv_bridge::CvImage> img_bridge_;
          img_bridge_ = cv_bridge::toCvCopy(this->image_msg_);

          // for debug
          //cvNamedWindow("showme",CV_WINDOW_AUTOSIZE);
          //cvSetMouseCallback("showme", &GazeboRosProsilica::mouse_cb, this);
          //cvStartWindowThread();
          //cvShowImage("showme",img_bridge_.toIpl());

          // crop image to roi
          cv::Mat roi(img_bridge_->image,
            cv::Rect(req.roi.x_offset, req.roi.y_offset,
                     req.roi.width, req.roi.height));
          img_bridge_->image = roi;

          // copy roi'd image into roiImageMsg
          img_bridge_->toImageMsg(*this->roiImageMsg);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  (*this->image_connect_count_)--;
  rsp.success = true;
  return;
}


/*
void GazeboRosProsilica::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  this->simTime  = msgs::Convert( _msg->sim_time() );

  ignition::math::Pose3d pose;
  pose.Pos().X() = 0.5*sin(0.01*this->simTime.Double());
  gzdbg << "plugin simTime [" << this->simTime.Double() << "] update pose [" << pose.Pos().X() << "]\n";
}
*/

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosProsilica)


}
