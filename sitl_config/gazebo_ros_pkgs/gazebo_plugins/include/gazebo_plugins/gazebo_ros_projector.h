/*
 * Desc: A dynamic controller plugin that controls texture projection.
 * Author: Jared Duke
 * Date: 17 June 2010
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_PROJECTOR_HH
#define GAZEBO_ROS_PROJECTOR_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/RenderTypes.hh>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <OgrePrerequisites.h>
#include <OgreTexture.h>
#include <OgreFrameListener.h>

namespace Ogre
{
  class PlaneBoundedVolumeListSceneQuery;
  class Frustum;
  class Pass;
  class SceneNode;
}

namespace gazebo
{

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosProjector Plugin XML Reference and Example

  \brief Ros Texture Projector Controller.

  This is a controller that controls texture projection into the world from a given body.

  Example Usage:
  \verbatim
  <projector name="projector_model">
    <body:empty name="projector_body_name">
     ...
      <controller:gazebo_ros_projector_controller name="projector_controller" plugin="libgazebo_ros_projector.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>15.0</updateRate>
        <textureName>stereo_projection_pattern_alpha.png</textureName>
        <filterTextureName>stereo_projection_pattern_filter.png</filterTextureName>
        <textureTopicName>projector_controller/image</textureTopicName>
        <projectorTopicName>projector_controller/projector</projectorTopicName>
        <fov>0.785398163</fov>
        <nearClipDist>0.1</nearClipDist>
        <farClipDist>10</farClipDist>
      </controller:gazebo_ros_projector_controller>

    </body:empty>
  </model:phyiscal>
  \endverbatim

\{
*/

class GazeboRosProjector : public ModelPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model
  public: GazeboRosProjector();

  /// \brief Destructor
  public: virtual ~GazeboRosProjector();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief pointer to the world
  private: physics::WorldPtr world_;

  /// \brief Callback when a texture is published
  private: void LoadImage(const std_msgs::String::ConstPtr& imageMsg);

  /// \brief Callbakc when a projector toggle is published
  private: void ToggleProjector(const std_msgs::Int32::ConstPtr& projectorMsg);

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber imageSubscriber_;
  private: ros::Subscriber projectorSubscriber_;

 /// \brief ROS texture topic name
  private: std::string texture_topic_name_;

  /// \brief ROS projector topic name
  private: std::string projector_topic_name_;

  /// \brief For setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

  private: event::ConnectionPtr add_model_event_;

  private: transport::NodePtr node_;
  private: transport::PublisherPtr projector_pub_;
};

/** \} */
/// @}

}
#endif

