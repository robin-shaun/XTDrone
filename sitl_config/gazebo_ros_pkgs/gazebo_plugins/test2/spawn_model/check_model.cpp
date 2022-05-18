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
#include "gazebo/GetWorldProperties.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"

#include <sstream>
#include <tinyxml.h>
#include <boost/lexical_cast.hpp>

// Including ros, just to be able to call ros::init(), to remove unwanted
// args from command-line.
#include <ros/ros.h>

int g_argc;
char** g_argv;

TEST(SpawnTest, spawnSingleBox)
{
  ros::NodeHandle nh("");
  // make the service call to spawn model
  ros::service::waitForService("gazebo/get_world_properties");
  ros::ServiceClient check_model_client = nh.serviceClient<gazebo::GetWorldProperties>("gazebo/get_world_properties");
  gazebo::GetWorldProperties world_properties;


  bool found = false;

  // test duration
  double test_duration = 10.0; // default to 10 seconds sim time
  try
  {
    test_duration = boost::lexical_cast<double>(g_argv[1]);
  }
  catch (boost::bad_lexical_cast &e)
  {
    ROS_ERROR_NAMED("check_model", "first argument of check_model should be timeout");
    return;
  }
  ros::Time timeout = ros::Time::now() + ros::Duration(test_duration);

  // model name to look for
  std::string model_name = std::string(g_argv[2]);
  ROS_INFO_NAMED("check_model", "looking for model: %s",model_name.c_str());

  while (!found && ros::Time::now() < timeout)
  {
    check_model_client.call(world_properties);
    for (std::vector<std::string>::iterator mit = world_properties.response.model_names.begin();
                                            mit != world_properties.response.model_names.end();
                                            mit++)
    {
      if (*mit == model_name)
      {
        found = true;
        break;
      }
    }
  }

  ASSERT_TRUE(found);

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
