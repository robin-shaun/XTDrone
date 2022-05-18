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

#include <dynamic_reconfigure/server.h>
#include <gazebo_plugins/HokuyoConfig.h>

void callback(gazebo_plugins::HokuyoConfig &config, uint32_t level)
{
  ROS_INFO_NAMED("hokuyo_node", "Reconfigure request : %f %f %i %i %i %s %i %s %f %i",
           config.min_ang, config.max_ang, (int)config.intensity, config.cluster, config.skip,
           config.port.c_str(), (int)config.calibrate_time, config.frame_id.c_str(), config.time_offset, (int)config.allow_unsafe_settings);

  // do nothing for now

  ROS_INFO_NAMED("hokuyo_node", "Reconfigure to : %f %f %i %i %i %s %i %s %f %i",
           config.min_ang, config.max_ang, (int)config.intensity, config.cluster, config.skip,
           config.port.c_str(), (int)config.calibrate_time, config.frame_id.c_str(), config.time_offset, (int)config.allow_unsafe_settings);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hokuyo_node");
  dynamic_reconfigure::Server<gazebo_plugins::HokuyoConfig> srv;
  dynamic_reconfigure::Server<gazebo_plugins::HokuyoConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  ROS_INFO_NAMED("hokuyo_node", "Starting to spin...");
  ros::spin();
  return 0;
}
