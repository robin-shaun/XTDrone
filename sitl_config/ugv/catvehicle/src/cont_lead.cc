/*
Author: Rahul Kumar Bhadani, Jonathan Sprinkle, Matt Bunting
Copyright (c) 2018 Arizona Board of Regents
All rights reserved.

Permission is hereby granted, without written agreement and without
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the
above copyright notice and the following two paragraphs appear in
all copies of this software.

IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

*/

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/physics/Joint.hh>
#include <stdlib.h>
//#include <unistd.h>

#include "cont_lead.hh"

namespace gazebo
{
 void LeadSteering::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
 {
      // Store the pointer to the model
      model = _parent;
      
//      prev_angle = 0.0;
      angle = 0.0;
//      cur_angle = 0.0;

	// Get Pointer to the for jointcontroller
		j_cont = new physics::JointController(model);

	//Get pointers at joints
        steering_joints[0] = model->GetJoint("front_left_steering_joint");
        steering_joints[1] = model->GetJoint("front_right_steering_joint");
	
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LeadSteering::OnUpdate, this, _1));


	//Start up ros_node
	std::string robot_namespace = "/";
	int argc = 0;
	char** argv = NULL;
	      ros::init(argc, argv, "lead_sim", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	      rosnode_ = new ros::NodeHandle(robot_namespace);
	      this->ros_spinner_thread_ = boost::thread( boost::bind( &LeadSteering::LeadVehicleSimROSThread,this ) );
 } //end Load

    // Called by the world update start event
 void LeadSteering::OnUpdate(const common::UpdateInfo & /*_info*/)
 {
//	SteeringSmooth();	
	j_cont->SetJointPosition(steering_joints[0], angle);
	j_cont->SetJointPosition(steering_joints[1], angle);
 } //end OnUpdate

 void LeadSteering::LeadVehicleSimROSThread()
 {
     ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
     std::string topic_name = "cmd_str_lead";
     sub_ = rosnode_->subscribe<geometry_msgs::Twist>(topic_name, 1000, &LeadSteering::Callback, this);

     while (this->rosnode_->ok())
     {
       usleep(1000);
       ros::spinOnce();
     }
 } //end LeadVehicleSimROSThread


 void LeadSteering::Callback(const geometry_msgs::Twist::ConstPtr& msg)
 {
	float ang = msg->angular.z;
	angle = ang;
//	cur_angle = ang;
//	SteeringSmooth();
 } //end Callback

/* void LeadSteering::SteeringSmooth()
 {
	double mini = 0.01;
	difference = cur_angle - prev_angle;

	if( ( abs(difference) > mini ) && ( difference > 0.0) )
	{
		prev_angle += mini;
		angle = prev_angle;
	        usleep(1000);
		SteeringSmooth();
	}
	else if( ( abs(difference) > mini ) && ( difference < 0.0) )
	{
	        prev_angle -= mini;
		angle = prev_angle;
	        usleep(2000);
		SteeringSmooth();
	}
	else
	{
		angle = cur_angle;
		prev_angle = angle;
	}
 }*/	
	
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeadSteering)
}
