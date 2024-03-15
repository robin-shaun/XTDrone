/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: A dynamic controller plugin that publishes ROS image_raw camera_info topic for generic camera sensor.
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef GAZEBO_ROS_OPENNI_KINECT_HH
#define GAZEBO_ROS_OPENNI_KINECT_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ros messages stuff
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>

// gazebo stuff
#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>

// dynamic reconfigure stuff
#include <gazebo_plugins/GazeboRosOpenniKinectConfig.h>
#include <dynamic_reconfigure/server.h>

// boost stuff
#include <boost/thread/mutex.hpp>

// camera stuff
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
  class GazeboRosOpenniKinect : public DepthCameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosOpenniKinect();

    /// \brief Destructor
    public: ~GazeboRosOpenniKinect();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Advertise point cloud and depth image
    public: virtual void Advertise();

    /// \brief Update the controller
    protected: virtual void OnNewDepthFrame(const float *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    protected: virtual void OnNewImageFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    /// \brief push point cloud data into ros topic
    private: void FillPointdCloud(const float *_src);

    /// \brief push depth image data into ros topic
    private: void FillDepthImage(const float *_src);

    /// \brief Keep track of number of connctions for point clouds
    private: int point_cloud_connect_count_;
    private: void PointCloudConnect();
    private: void PointCloudDisconnect();

    /// \brief Keep track of number of connctions for point clouds
    private: int depth_image_connect_count_;
    private: void DepthImageConnect();
    private: void DepthImageDisconnect();

    private: bool FillPointCloudHelper(sensor_msgs::PointCloud2 &point_cloud_msg,
                                  uint32_t rows_arg, uint32_t cols_arg,
                                  uint32_t step_arg, void* data_arg);

    private: bool FillDepthImageHelper( sensor_msgs::Image& image_msg,
                                  uint32_t height, uint32_t width,
                                  uint32_t step, void* data_arg);

    /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
    private: ros::Publisher point_cloud_pub_;
    private: ros::Publisher depth_image_pub_;

    /// \brief PointCloud2 point cloud message
    private: sensor_msgs::PointCloud2 point_cloud_msg_;
    private: sensor_msgs::Image depth_image_msg_;

    /// \brief Minimum range of the point cloud
    private: double point_cloud_cutoff_;
    /// \brief Maximum range of the point cloud
    private: double point_cloud_cutoff_max_;

    /// \brief ROS image topic name
    private: std::string point_cloud_topic_name_;


    /// \brief image where each pixel contains the depth data
    private: std::string depth_image_topic_name_;
    private: common::Time depth_sensor_update_time_;

    // overload with our own
    private: std::string depth_image_camera_info_topic_name_;
    private: int depth_info_connect_count_;
    private: void DepthInfoConnect();
    private: void DepthInfoDisconnect();
    private: common::Time last_depth_image_camera_info_update_time_;
    private: bool use_depth_image_16UC1_format_;
    protected: ros::Publisher depth_image_camera_info_pub_;

    using GazeboRosCameraUtils::PublishCameraInfo;
    protected: virtual void PublishCameraInfo();

    private: event::ConnectionPtr load_connection_;
  };

}
#endif

