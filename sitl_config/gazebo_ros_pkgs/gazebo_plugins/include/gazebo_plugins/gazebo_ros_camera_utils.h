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

#ifndef GAZEBO_ROS_CAMERA_UTILS_HH
#define GAZEBO_ROS_CAMERA_UTILS_HH

#include <string>
// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ros messages stuff
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// dynamic reconfigure stuff
#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>

// Gazebo
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo
{
  class GazeboRosMultiCamera;
  class GazeboRosTriggeredMultiCamera;
  class GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosCameraUtils();

    /// \brief Destructor
    public: ~GazeboRosCameraUtils();

    /// \brief Load the plugin.
    /// \param[in] _parent Take in SDF root element.
    /// \param[in] _sdf SDF values.
    /// \param[in] _camera_name_suffix required before calling LoadThread
    public: void Load(sensors::SensorPtr _parent,
                      sdf::ElementPtr _sdf,
                      const std::string &_camera_name_suffix = "");

    /// \brief Load the plugin.
    /// \param[in] _parent Take in SDF root element.
    /// \param[in] _sdf SDF values.
    /// \param[in] _camera_name_suffix Suffix of the camera name.
    /// \param[in] _hack_baseline Multiple camera baseline.
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf,
                      const std::string &_camera_name_suffix,
                      double _hack_baseline);

    public: event::ConnectionPtr OnLoad(const boost::function<void()>&);

    private: void Init();

    /// \brief Put camera data to the ROS topic
    protected: void PutCameraData(const unsigned char *_src);
    protected: void PutCameraData(const unsigned char *_src,
      common::Time &last_update_time);

    /// \brief Keep track of number of image connections
    protected: boost::shared_ptr<int> image_connect_count_;
    /// \brief A mutex to lock access to image_connect_count_
    protected: boost::shared_ptr<boost::mutex> image_connect_count_lock_;
    protected: void ImageConnect();
    protected: void ImageDisconnect();

    /// \brief Keep track when we activate this camera through ros
    /// subscription, was it already active?  resume state when
    /// unsubscribed.
    protected: boost::shared_ptr<bool> was_active_;

    /// \brief: Camera modification functions
    private: void SetHFOV(const std_msgs::Float64::ConstPtr& hfov);
    private: void SetUpdateRate(const std_msgs::Float64::ConstPtr& update_rate);

    /// \brief A pointer to the ROS node.
    ///  A node will be instantiated if it does not exist.
    protected: ros::NodeHandle* rosnode_;
    protected: image_transport::Publisher image_pub_;
    private: image_transport::ImageTransport* itnode_;

    /// \brief ROS image message
    protected: sensor_msgs::Image image_msg_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    /// \brief ROS camera name
    private: std::string camera_name_;

    /// \brief tf prefix
    private: std::string tf_prefix_;

    /// \brief ROS image topic name
    protected: std::string image_topic_name_;

    /// \brief Publish CameraInfo to the ROS topic
    protected: void PublishCameraInfo(ros::Publisher camera_info_publisher);
    protected: void PublishCameraInfo(common::Time &last_update_time);
    protected: void PublishCameraInfo();
    /// \brief Keep track of number of connctions for CameraInfo
    private: void InfoConnect();
    private: void InfoDisconnect();
    /// \brief camera info
    protected: ros::Publisher camera_info_pub_;
    protected: std::string camera_info_topic_name_;
    protected: common::Time last_info_update_time_;

    /// \brief ROS frame transform name to use in the image message header.
    ///        This should typically match the link name the sensor is attached.
    protected: std::string frame_name_;
    /// update rate of this sensor
    protected: double update_rate_;
    protected: double update_period_;
    protected: common::Time last_update_time_;

    protected: double cx_prime_;
    protected: double cx_;
    protected: double cy_;
    protected: double focal_length_;
    protected: double hack_baseline_;
    protected: double distortion_k1_;
    protected: double distortion_k2_;
    protected: double distortion_k3_;
    protected: double distortion_t1_;
    protected: double distortion_t2_;

    protected: bool auto_distortion_;
    protected: bool border_crop_;

    protected: boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;


    /// \brief A mutex to lock access to fields
    /// that are used in ROS message callbacks
    protected: boost::mutex lock_;

    /// \brief size of image buffer
    protected: std::string type_;
    protected: int skip_;

    private: ros::Subscriber cameraHFOVSubscriber_;
    private: ros::Subscriber cameraUpdateRateSubscriber_;

    // Time last published, refrain from publish unless new image has
    // been rendered
    // Allow dynamic reconfiguration of camera params
    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
      *dyn_srv_;
    void configCallback(gazebo_plugins::GazeboRosCameraConfig &config,
      uint32_t level);

    protected: ros::CallbackQueue camera_queue_;
    protected: void CameraQueueThread();
    protected: boost::thread callback_queue_thread_;


    // copied from CameraPlugin
    protected: unsigned int width_, height_, depth_;
    protected: std::string format_;

    protected: sensors::SensorPtr parentSensor_;
    protected: rendering::CameraPtr camera_;

    // Pointer to the world
    protected: physics::WorldPtr world_;

    private: event::ConnectionPtr newFrameConnection_;

    protected: common::Time sensor_update_time_;

    // maintain for one more release for backwards compatibility
    protected: physics::WorldPtr world;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: event::EventT<void()> load_event_;

    // make a trigger function that the child classes can override
    // and a function that returns bool to indicate whether the trigger
    // should be used
    protected: virtual void TriggerCamera();
    protected: virtual bool CanTriggerCamera();
    private: void TriggerCameraInternal(const std_msgs::Empty::ConstPtr &dummy);
    private: ros::Subscriber trigger_subscriber_;

    /// \brief ROS trigger topic name
    protected: std::string trigger_topic_name_;

    /// \brief True if camera util is initialized
    protected: bool initialized_;

    friend class GazeboRosMultiCamera;
    friend class GazeboRosTriggeredMultiCamera;
  };
}
#endif
