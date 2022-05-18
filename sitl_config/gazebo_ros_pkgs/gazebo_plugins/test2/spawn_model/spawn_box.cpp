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
#include "gazebo/GetWorldProperties.h"
#include "gazebo/DeleteModel.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"

#include <sstream>
#include <tinyxml.h>

// Including ros, just to be able to call ros::init(), to remove unwanted
// args from command-line.
#include <ros/ros.h>

int g_argc;
char** g_argv;

TEST(SpawnTest, spawnSingleBox)
{
  ros::NodeHandle nh("");
  // make the service call to spawn model
  ros::service::waitForService("gazebo/spawn_urdf_model");
  ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo::SpawnModel>("gazebo/spawn_urdf_model");
  gazebo::SpawnModel spawn_model;
  spawn_model.request.model_name = "box1";

  // load urdf file
  std::string urdf_filename = std::string(g_argv[1]);
  ROS_DEBUG_NAMED("spawn_box", "loading file: %s",urdf_filename.c_str());
  // read urdf / gazebo model xml from file
  TiXmlDocument xml_in(urdf_filename);
  xml_in.LoadFile();
  std::ostringstream stream;
  stream << xml_in;
  spawn_model.request.model_xml = stream.str(); // load xml file
  ROS_DEBUG_NAMED("spawn_box", "XML string: %s",stream.str().c_str());

  spawn_model.request.robot_namespace = "";
  geometry_msgs::Pose pose;
  pose.position.x = pose.position.y = 0; pose.position.z = 1;
  pose.orientation.w = 1.0; pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
  spawn_model.request.initial_pose = pose;
  spawn_model.request.reference_frame = "";

  ASSERT_TRUE(spawn_model_client.call(spawn_model));

  SUCCEED();
}


TEST(SpawnTest, spawnBoxStack)
{
  ros::NodeHandle nh("");
  // make the service call to spawn model
  ros::service::waitForService("gazebo/spawn_urdf_model");
  ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo::SpawnModel>("gazebo/spawn_urdf_model");
  gazebo::SpawnModel spawn_model;

  // load urdf file
  std::string urdf_filename = std::string(g_argv[1]);
  ROS_DEBUG_NAMED("spawn_box", "loading file: %s",urdf_filename.c_str());
  // read urdf / gazebo model xml from file
  TiXmlDocument xml_in(urdf_filename);
  xml_in.LoadFile();
  std::ostringstream stream;
  stream << xml_in;
  spawn_model.request.model_xml = stream.str(); // load xml file
  ROS_DEBUG_NAMED("spawn_box", "XML string: %s",stream.str().c_str());

  spawn_model.request.robot_namespace = "";
  spawn_model.request.reference_frame = "";

  // spawn 10 boxes
  for (int i=0;i<10;i++)
  {
    std::ostringstream mn_stream;
    mn_stream << "box_" << i;
    spawn_model.request.model_name = mn_stream.str();
    geometry_msgs::Pose pose;
    pose.position.x = pose.position.y = 0; pose.position.z = 1 + 2.0*(i+1);
    pose.orientation.w = 1.0; pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
    spawn_model.request.initial_pose = pose;
    ASSERT_TRUE(spawn_model_client.call(spawn_model));
  }

  SUCCEED();
}

TEST(DeleteTest, deleteAllModels)
{
  ros::NodeHandle nh("");
  // make the service call to delete model
  ros::service::waitForService("gazebo/delete_model");
  ros::ServiceClient delete_model_client = nh.serviceClient<gazebo::DeleteModel>("gazebo/delete_model");
  gazebo::DeleteModel delete_model;

  // model names to delete
  ros::service::waitForService("gazebo/get_world_properties");
  ros::ServiceClient check_model_client = nh.serviceClient<gazebo::GetWorldProperties>("gazebo/get_world_properties");
  gazebo::GetWorldProperties world_properties;
  // get all models in the world and loop through deleting each one
  check_model_client.call(world_properties);
  for (std::vector<std::string>::iterator mit = world_properties.response.model_names.begin();
                                          mit != world_properties.response.model_names.end();
                                          mit++)
  {
    delete_model.request.model_name = *mit;
    ASSERT_TRUE(delete_model_client.call(delete_model));
  }

  // should have no more models in the world
  check_model_client.call(world_properties);
  ASSERT_TRUE(world_properties.response.model_names.empty());

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
