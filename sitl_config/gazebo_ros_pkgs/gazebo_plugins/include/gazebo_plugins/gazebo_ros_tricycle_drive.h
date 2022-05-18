/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * \file  gazebo_ros_tricycle_drive.cpp
 * \brief A tricycle drive plugin for gazebo.
 * \author  Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of June 2014
 */

#ifndef TRICYCLE_DRIVE_PLUGIN_HH
#define TRICYCLE_DRIVE_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

class Joint;
class Entity;


class GazeboRosTricycleDrive : public ModelPlugin {

    class TricycleDriveCmd {
    public:
        TricycleDriveCmd():speed(0), angle(0) {};
        double speed;
        double angle;
    };

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
public:
    GazeboRosTricycleDrive();
    ~GazeboRosTricycleDrive();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();
    virtual void FiniChild();

private:
    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr parent;
    void publishOdometry(double step_time);
    void publishWheelTF(); /// publishes the wheel tf's
    void publishWheelJointState();
    void motorController(double target_speed, double target_angle, double dt);

    event::ConnectionPtr update_connection_;

    physics::JointPtr joint_steering_;
    physics::JointPtr joint_wheel_actuated_;
    physics::JointPtr joint_wheel_encoder_left_;
    physics::JointPtr joint_wheel_encoder_right_;

    double diameter_encoder_wheel_;
    double diameter_actuated_wheel_;
    double wheel_acceleration_;
    double wheel_deceleration_;
    double wheel_speed_tolerance_;
    double steering_angle_tolerance_;
    double steering_speed_;
    double separation_encoder_wheel_;

    OdomSource odom_source_;
    double wheel_torque_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;

    // ROS STUFF
    ros::Publisher odometry_publisher_;
    ros::Subscriber cmd_vel_subscriber_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    nav_msgs::Odometry odom_;

    boost::mutex lock;


    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    /// updates the relative robot pose based on the wheel encoders
    void UpdateOdometryEncoder();

    TricycleDriveCmd cmd_;
    bool alive_;
    geometry_msgs::Pose2D pose_encoder_;
    common::Time last_odom_update_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_actuator_update_;

    // Flags
    bool publishWheelTF_;
    bool publishWheelJointState_;

};

}

#endif

