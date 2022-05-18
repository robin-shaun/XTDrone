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

#ifndef VISION_RECONFIGURE_HH
#define VISION_RECONFIGURE_HH

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <gazebo_plugins/CameraSynchronizerConfig.h>

#include <ros/callback_queue.h>

class VisionReconfigure
{
  public:
    VisionReconfigure();

    ~VisionReconfigure();

    void ReconfigureCallback(gazebo_plugins::CameraSynchronizerConfig &config, uint32_t level);
    void QueueThread();
    void spinOnce();
    void spin(double spin_frequency);

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_projector_;
    ros::Publisher pub_header_;
    dynamic_reconfigure::Server<gazebo_plugins::CameraSynchronizerConfig> srv_;
    std_msgs::Int32 projector_msg_;
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;

};

#endif
