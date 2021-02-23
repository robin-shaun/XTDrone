/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef ROBOTX_GAZEBO_TEST_HELPERS_HH_
#define ROBOTX_GAZEBO_TEST_HELPERS_HH_

#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <string>

/// \brief Check whether a model exists on simulation.
/// \param[in] _name The model name.
/// \param[in] _timeout Maximum timeout to wait for a model.
/// \return True when the model was found or false otherwise.
bool ModelExists(const std::string &_name,
                 const ros::WallDuration _timeout = ros::WallDuration(5, 0))
{
  ros::WallTime timeout = ros::WallTime::now() + _timeout;
  while (ros::WallTime::now() < timeout)
  {
    gazebo_msgs::ModelStatesConstPtr modelStates =
      ros::topic::waitForMessage<gazebo_msgs::ModelStates>(
        std::string("/gazebo/model_states"), ros::Duration(0.1));

    if (!modelStates) continue;
    for (auto model : modelStates->name)
    {
      if (model == _name)
        return true;
    }
  }
  return false;
}
#endif
