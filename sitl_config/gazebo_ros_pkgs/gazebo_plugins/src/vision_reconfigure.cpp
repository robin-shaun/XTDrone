/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009-2010, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
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

#include <gazebo_plugins/vision_reconfigure.h>

VisionReconfigure::VisionReconfigure() : nh_("")
{
  this->nh_.setCallbackQueue(&this->queue_);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &VisionReconfigure::QueueThread,this ) );

  // this code needs to be rewritten
  // for now, it publishes on pub_projector_ which is used by gazebo_ros_projector plugin directly
  //          and it publishes pub_header_, which is published by projector_controller in ethercat_trigger_controllers package in real life
  this->pub_projector_ = this->nh_.advertise<std_msgs::Int32>("/projector_wg6802418_controller/projector", 1,true); // publish latched for sim
  this->pub_header_ = this->nh_.advertise<std_msgs::Header>("/projector_controller/rising_edge_timestamps", 1,true); // publish latched for sim
  dynamic_reconfigure::Server<gazebo_plugins::CameraSynchronizerConfig>::CallbackType f = boost::bind(&VisionReconfigure::ReconfigureCallback, this, _1, _2);
  this->srv_.setCallback(f);


  // initialize from relevant params on server
  gazebo_plugins::CameraSynchronizerConfig config;
  this->nh_.getParam("/camera_synchronizer_node/projector_mode",config.projector_mode);
  this->nh_.getParam("/camera_synchronizer_node/forearm_l_trig_mode",config.forearm_l_trig_mode);
  this->nh_.getParam("/camera_synchronizer_node/forearm_r_trig_mode",config.forearm_r_trig_mode);
  this->nh_.getParam("/camera_synchronizer_node/narrow_stereo_trig_mode",config.narrow_stereo_trig_mode);
  this->nh_.getParam("/camera_synchronizer_node/wide_stereo_trig_mode",config.wide_stereo_trig_mode);
  this->ReconfigureCallback(config,0);

}

VisionReconfigure::~VisionReconfigure()
{
  this->nh_.shutdown();
  this->callback_queue_thread_.join();
}

void VisionReconfigure::ReconfigureCallback(gazebo_plugins::CameraSynchronizerConfig &config, uint32_t level)
{

  // turn on or off projector in gazebo plugin
  if (config.projector_mode == gazebo_plugins::CameraSynchronizer_ProjectorOff)
  {
    this->projector_msg_.data = 0;
  }
  else if (config.projector_mode == gazebo_plugins::CameraSynchronizer_ProjectorAuto)
  {
    // turn on or off projector based on narrow stereo trigger mode
    if (config.wide_stereo_trig_mode == gazebo_plugins::CameraSynchronizer_WithProjector ||
        config.narrow_stereo_trig_mode == gazebo_plugins::CameraSynchronizer_WithProjector ||
        config.forearm_r_trig_mode == gazebo_plugins::CameraSynchronizer_WithProjector ||
        config.forearm_l_trig_mode == gazebo_plugins::CameraSynchronizer_WithProjector)
    {
      this->projector_msg_.data = 1;
    }
    else if (config.wide_stereo_trig_mode == gazebo_plugins::CameraSynchronizer_AlternateProjector ||
             config.narrow_stereo_trig_mode == gazebo_plugins::CameraSynchronizer_AlternateProjector ||
             config.forearm_r_trig_mode == gazebo_plugins::CameraSynchronizer_AlternateProjector ||
             config.forearm_l_trig_mode == gazebo_plugins::CameraSynchronizer_AlternateProjector)
    {
      ROS_WARN_NAMED("vision_reconfigure", "Alternate Projector Mode not supported in simulation, setting projector to on for now");
      this->projector_msg_.data = 1;
    }
    else
    {
      ROS_DEBUG_NAMED("vision_reconfigure", "Projector only supported for modes: WithProjector and AlternateProjector");
      this->projector_msg_.data = 0;
    }
  }
  else if (config.projector_mode == gazebo_plugins::CameraSynchronizer_ProjectorOn)
  {
    this->projector_msg_.data = 1;
  }
  else
  {
    ROS_ERROR_NAMED("vision_reconfigure", "projector_mode is not in any recognized state [%d]",config.projector_mode);
  }

  this->pub_projector_.publish(projector_msg_);
}

void VisionReconfigure::QueueThread()
{
  // FIXME: hardcoded to 100Hz update rate for ros callback queue
  static const double timeout = 0.01;
  while (this->nh_.ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void VisionReconfigure::spinOnce()
{
  if (projector_msg_.data == 1)
  {
    std_msgs::Header rh;
    rh.stamp = ros::Time::now();
    rh.frame_id = "projector_wg6802418_frame";
    this->pub_header_.publish(rh);
  }
}

void VisionReconfigure::spin(double spin_frequency)
{
  ros::Rate loop_rate(spin_frequency);
  while(this->nh_.ok())
  {
    ros::spinOnce();
    this->spinOnce();
    loop_rate.sleep();
  }
}
