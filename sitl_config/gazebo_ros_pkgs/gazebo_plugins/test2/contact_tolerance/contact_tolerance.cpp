/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: John Hsu */

#include <string>
#include <gtest/gtest.h>
#include "gazebo/SpawnModel.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "std_srvs/Empty.h"
#include "gazebo/GetModelState.h"

#include <sstream>
#include <tinyxml.h>

// Including ros, just to be able to call ros::init(), to remove unwanted
// args from command-line.
#include <ros/ros.h>

int g_argc;
char** g_argv;

TEST(SpawnTest, checkBoxStackDrift)
{
  ros::NodeHandle nh("");
  // make the service call to spawn model
  ros::service::waitForService("gazebo/spawn_urdf_model");
  ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo::SpawnModel>("gazebo/spawn_urdf_model");
  gazebo::SpawnModel spawn_model;

  // load urdf file
  std::string urdf_filename = std::string(g_argv[1]);
  ROS_DEBUG_NAMED("contact_tolerance", "loading file: %s",urdf_filename.c_str());
  // read urdf / gazebo model xml from file
  TiXmlDocument xml_in(urdf_filename);
  xml_in.LoadFile();
  std::ostringstream stream;
  stream << xml_in;
  spawn_model.request.model_xml = stream.str(); // load xml file
  ROS_DEBUG_NAMED("contact_tolerance", "XML string: %s",stream.str().c_str());

  spawn_model.request.robot_namespace = "";
  spawn_model.request.reference_frame = "";

  // spawn 10 boxes
  for (int i=0;i<10;i++)
  {
    std::ostringstream mn_stream;
    mn_stream << "box_" << i;
    spawn_model.request.model_name = mn_stream.str();
    geometry_msgs::Pose pose;
    pose.position.x = pose.position.y = 0; pose.position.z = 1 + 2.0*(i);
    pose.orientation.w = 1.0; pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
    spawn_model.request.initial_pose = pose;
    ASSERT_TRUE(spawn_model_client.call(spawn_model));
  }

  // get pose of top box, check for drift
  // setup the service call to get model state
  ros::service::waitForService("gazebo/get_model_state");
  ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo::GetModelState>("gazebo/get_model_state");
  gazebo::GetModelState model_state;
  model_state.request.model_name = "box_9";
  double test_duration = 10.0; // check drift over this period of sim time
  double LINEAR_DRIFT_TOLERANCE = 0.0105; // linear drift tolerance of top block
                                          // given contact layer is 1mm, 10 blocks leads to 10*1mm and .5mm of numerical error
  ros::Time timeout = ros::Time::now() + ros::Duration(test_duration);
  // get intial pose
  ASSERT_TRUE(get_model_state_client.call(model_state));
  geometry_msgs::Pose initial_pose = model_state.response.pose;
  while (ros::Time::now() < timeout)
  {
    ASSERT_TRUE(get_model_state_client.call(model_state));
    geometry_msgs::Pose current_pose = model_state.response.pose;
    double error_x = fabs(current_pose.position.x - initial_pose.position.x);
    double error_y = fabs(current_pose.position.y - initial_pose.position.y);
    double error_z = fabs(current_pose.position.z - initial_pose.position.z);
    double error_linear = sqrt(error_x*error_x+error_y*error_y+error_z*error_z);
    ROS_INFO_NAMED("contact_tolerance", "error: %f",error_linear);
    if (error_linear > LINEAR_DRIFT_TOLERANCE);
      ROS_INFO_NAMED("contact_tolerance", "box stack teset failed with this linear error (%f):  this means the top box in the box stack is moving too much.  Check to see if surface contact layer has changed from 1mm, bounce parameter has changed, world time step, solver has changed.  You can reproduce this test by:\n\nroslaunch gazebo_tests empty.launch\nrosrun gazebo_tests contact_tolerance `rospack find gazebo_tests`/test/urdf/box.urdf\n",error_linear);
    ASSERT_TRUE(error_linear <= LINEAR_DRIFT_TOLERANCE);

    // FIXME: do something about orientation drift too
    geometry_msgs::Quaternion current_q = current_pose.orientation;
    geometry_msgs::Quaternion initial_q = initial_pose.orientation;
  }


  SUCCEED();
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
