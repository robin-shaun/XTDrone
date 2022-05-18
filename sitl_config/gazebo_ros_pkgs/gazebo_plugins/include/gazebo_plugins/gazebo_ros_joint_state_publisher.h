/*
 * Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/


#ifndef JOINT_STATE_PUBLISHER_PLUGIN_HH
#define JOINT_STATE_PUBLISHER_PLUGIN_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

// Usage in URDF:
//   <gazebo>
//       <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
// 		<robotNamespace>/pioneer2dx</robotNamespace>
// 		<jointName>chassis_swivel_joint, swivel_wheel_joint, left_hub_joint, right_hub_joint</jointName>
// 		<updateRate>100.0</updateRate>
// 		<alwaysOn>true</alwaysOn>
//       </plugin>
//   </gazebo>



namespace gazebo {
class GazeboRosJointStatePublisher : public ModelPlugin {
public:
    GazeboRosJointStatePublisher();
    ~GazeboRosJointStatePublisher();
    void Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
    void OnUpdate ( const common::UpdateInfo & _info );
    void publishJointStates();
    // Pointer to the model
private:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world_;
    physics::ModelPtr parent_;
    std::vector<physics::JointPtr> joints_;

    // ROS STUFF
    boost::shared_ptr<ros::NodeHandle> rosnode_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    std::string tf_prefix_;
    std::string robot_namespace_;
    std::vector<std::string> joint_names_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN ( GazeboRosJointStatePublisher )
}

#endif //JOINT_STATE_PUBLISHER_PLUGIN_HH

