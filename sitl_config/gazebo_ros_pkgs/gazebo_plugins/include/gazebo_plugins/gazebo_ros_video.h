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
 * Desc: Video plugin for displaying ROS image topics on Ogre textures
 * Author: Piyush Khandelwal
 * Date: 26 July 2013
 */

#ifndef GAZEBO_ROS_VIDEO_H
#define GAZEBO_ROS_VIDEO_H

#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace gazebo
{

  class VideoVisual : public rendering::Visual
  {
    public:
      VideoVisual(
          const std::string &name, rendering::VisualPtr parent,
          int height, int width);
      virtual ~VideoVisual();
      void render(const cv::Mat& image);
    private:
      Ogre::TexturePtr texture_;
      int height_;
      int width_;
  };

  class GazeboRosVideo : public VisualPlugin
  {
    public:

      GazeboRosVideo();
      virtual ~GazeboRosVideo();

      void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf);
      void processImage(const sensor_msgs::ImageConstPtr &msg);

    protected:

      virtual void UpdateChild();

      // Pointer to the model
      rendering::VisualPtr model_;
      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<VideoVisual> video_visual_;

      cv_bridge::CvImagePtr image_;
      boost::mutex m_image_;
      bool new_image_available_;

      /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
      ros::NodeHandle* rosnode_;

      // ROS Stuff
      ros::Subscriber camera_subscriber_;
      std::string robot_namespace_;
      std::string topic_name_;

      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

  };

}

#endif
