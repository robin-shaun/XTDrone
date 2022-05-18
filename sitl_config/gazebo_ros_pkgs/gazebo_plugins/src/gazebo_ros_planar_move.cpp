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
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */

#include <gazebo_plugins/gazebo_ros_planar_move.h>

namespace gazebo
{

  GazeboRosPlanarMove::GazeboRosPlanarMove() {}

  GazeboRosPlanarMove::~GazeboRosPlanarMove() {}

  // Load the controller
  void GazeboRosPlanarMove::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf)
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace"))
    {
      ROS_INFO_NAMED("planar_move", "PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else
    {
      robot_namespace_ =
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), command_topic_.c_str());
    }
    else
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_topic_.c_str());
    }
    else
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    }
    else
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    }
    else
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }

#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
#endif
#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_pose_ = parent_->GetWorldPose().Ign();
#endif
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("planar_move", "PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG_NAMED("planar_move", "OCPlugin (%s) has started",
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosPlanarMove::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosPlanarMove::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosPlanarMove::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosPlanarMove::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(lock);
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = parent_->WorldPose();
#else
    ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
#endif
    float yaw = pose.Rot().Yaw();
    parent_->SetLinearVel(ignition::math::Vector3d(
          x_ * cosf(yaw) - y_ * sinf(yaw),
          y_ * cosf(yaw) + x_ * sinf(yaw),
          0));
    parent_->SetAngularVel(ignition::math::Vector3d(0, 0, rot_));
    if (odometry_rate_ > 0.0) {
#if GAZEBO_MAJOR_VERSION >= 8
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
      double seconds_since_last_update =
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosPlanarMove::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosPlanarMove::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg)
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
  }

  void GazeboRosPlanarMove::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosPlanarMove::publishOdometry(double step_time)
  {

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent_->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent_->GetWorldPose().Ign();
#endif

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3    vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame,
            base_footprint_frame));

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
    linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
    if (rot_ > M_PI / step_time)
    {
      // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = rot_;
    }
    else
    {
      float last_yaw = last_odom_pose_.Rot().Yaw();
      float current_yaw = pose.Rot().Yaw();
      while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
      while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_pub_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosPlanarMove)
}
