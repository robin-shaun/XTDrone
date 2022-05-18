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

#include <gazebo_plugins/gazebo_ros_video.h>
#include <boost/lexical_cast.hpp>

namespace gazebo
{

  VideoVisual::VideoVisual(
      const std::string &name, rendering::VisualPtr parent,
      int height, int width) :
      rendering::Visual(name, parent), height_(height), width_(width)
  {

    texture_ = Ogre::TextureManager::getSingleton().createManual(
        name + "__VideoTexture__",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        width_, height_,
        0,
        Ogre::PF_BYTE_BGRA,
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(
          name + "__VideoMaterial__", "General");
    material->getTechnique(0)->getPass(0)->createTextureUnitState(
        name + "__VideoTexture__");
    material->setReceiveShadows(false);

    double factor = 1.0;

    Ogre::ManualObject mo(name + "__VideoObject__");
    mo.begin(name + "__VideoMaterial__",
        Ogre::RenderOperation::OT_TRIANGLE_LIST);

    mo.position(-factor / 2, factor / 2, 0.51);
    mo.textureCoord(0, 0);

    mo.position(factor / 2, factor / 2, 0.51);
    mo.textureCoord(1, 0);

    mo.position(factor / 2, -factor / 2, 0.51);
    mo.textureCoord(1, 1);

    mo.position(-factor / 2, -factor / 2, 0.51);
    mo.textureCoord(0, 1);

    mo.triangle(0, 3, 2);
    mo.triangle(2, 1, 0);
    mo.end();

    mo.convertToMesh(name + "__VideoMesh__");

    Ogre::MovableObject *obj = (Ogre::MovableObject*)
      GetSceneNode()->getCreator()->createEntity(
          name + "__VideoEntity__",
          name + "__VideoMesh__");
    obj->setCastShadows(false);
    AttachObject(obj);
  }

  VideoVisual::~VideoVisual() {}

  void VideoVisual::render(const cv::Mat& image)
  {

    // Fix image size if necessary
    const cv::Mat* image_ptr = &image;
    cv::Mat converted_image;
    if (image_ptr->rows != height_ || image_ptr->cols != width_)
    {
      cv::resize(*image_ptr, converted_image, cv::Size(width_, height_));
      image_ptr = &converted_image;
    }

    // Get the pixel buffer
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer =
      texture_->getBuffer();

    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);

    memcpy(pDest, image_ptr->data, height_ * width_ * 4);

    // Unlock the pixel buffer
    pixelBuffer->unlock();
  }

  // Constructor
  GazeboRosVideo::GazeboRosVideo() {}

  // Destructor
  GazeboRosVideo::~GazeboRosVideo() {
    update_connection_.reset();

    // Custom Callback Queue
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();

    delete rosnode_;
  }

  // Load the controller
  void GazeboRosVideo::Load(
      rendering::VisualPtr parent, sdf::ElementPtr sdf)
  {

    model_ = parent;
    sdf::ElementPtr p_sdf;
    if (sdf->HasElement("sdf"))
    {
      p_sdf = sdf->GetElement("sdf");
    }
    else
    {
      p_sdf = sdf;
    }

    robot_namespace_ = "";
    if (!p_sdf->HasElement("robotNamespace"))
    {
      ROS_WARN_NAMED("video", "GazeboRosVideo plugin missing <robotNamespace>, "
          "defaults to \"%s\".", robot_namespace_.c_str());
    }
    else
    {
      robot_namespace_ =
        p_sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    topic_name_ = "image_raw";
    if (!p_sdf->HasElement("topicName"))
    {
      ROS_WARN_NAMED("video", "GazeboRosVideo Plugin (ns = %s) missing <topicName>, "
          "defaults to \"%s\".", robot_namespace_.c_str(), topic_name_.c_str());
    }
    else
    {
      topic_name_ = p_sdf->GetElement("topicName")->Get<std::string>();
    }

    int height = 240;
    if (!p_sdf->HasElement("height")) {
      ROS_WARN_NAMED("video", "GazeboRosVideo Plugin (ns = %s) missing <height>, "
          "defaults to %i.", robot_namespace_.c_str(), height);
    }
    else
    {
      height = p_sdf->GetElement("height")->Get<int>();
    }

    int width = 320;
    if (!p_sdf->HasElement("width")) {
      ROS_WARN_NAMED("video", "GazeboRosVideo Plugin (ns = %s) missing <width>, "
          "defaults to %i", robot_namespace_.c_str(), width);
    }
    else
    {
      width = p_sdf->GetElement("width")->Get<int>();
    }

    std::string name = robot_namespace_ + "_visual";
    video_visual_.reset(
        new VideoVisual(name, parent, height, width));

    // Initialize the ROS node for the gazebo client if necessary
    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "gazebo_client",
          ros::init_options::NoSigintHandler);
    }
    std::string gazebo_source =
      (ros::this_node::getName() == "/gazebo_client") ? "gzclient" : "gzserver";
    rosnode_ = new ros::NodeHandle(robot_namespace_);

    // Subscribe to the image topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<sensor_msgs::Image>(topic_name_, 1,
          boost::bind(&GazeboRosVideo::processImage, this, _1),
          ros::VoidPtr(), &queue_);
    camera_subscriber_ = rosnode_->subscribe(so);

    new_image_available_ = false;

    callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosVideo::QueueThread, this));

    update_connection_ =
      event::Events::ConnectPreRender(
          boost::bind(&GazeboRosVideo::UpdateChild, this));

    ROS_INFO_NAMED("video", "GazeboRosVideo (%s, ns = %s) has started",
        gazebo_source.c_str(), robot_namespace_.c_str());
  }

  // Update the controller
  void GazeboRosVideo::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(m_image_);
    if (new_image_available_)
    {
      video_visual_->render(image_->image);
    }
    new_image_available_ = false;
  }

  void GazeboRosVideo::processImage(const sensor_msgs::ImageConstPtr &msg)
  {
    // Get a reference to the image from the image message pointer
    boost::mutex::scoped_lock scoped_lock(m_image_);
    // We get image with alpha channel as it allows memcpy onto ogre texture
    image_ = cv_bridge::toCvCopy(msg, "bgra8");
    new_image_available_ = true;
  }

  void GazeboRosVideo::QueueThread()
  {
    static const double timeout = 0.01;
    while (rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_VISUAL_PLUGIN(GazeboRosVideo);
}
