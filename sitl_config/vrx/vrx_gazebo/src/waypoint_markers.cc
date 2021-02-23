/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "vrx_gazebo/waypoint_markers.hh"

/////////////////////////////////////////////////
WaypointMarkers::WaypointMarkers(std::string _namespace)
  : ns(_namespace), material("Gazebo/Red"), scaling{0.2, 0.2, 1.5}, height(4.0)
{
}

/////////////////////////////////////////////////
void WaypointMarkers::Load(sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("material"))
  {
    this->material = _sdf->Get<std::string>("material");
  }

  if (_sdf->HasElement("scaling"))
  {
    this->scaling = _sdf->Get<ignition::math::Vector3d>("scaling");
  }

  if (_sdf->HasElement("height"))
  {
    this->height = _sdf->Get<double>("height");
  }
}

/////////////////////////////////////////////////
bool WaypointMarkers::IsAvailable()
{
#if GAZEBO_MAJOR_VERSION >= 8
  return true;
#else
  return false;
#endif
}

/////////////////////////////////////////////////
bool WaypointMarkers::DrawMarker(int _marker_id, double _x, double _y,
    double _yaw, std::string _text)
{
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns(this->ns);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name(this->material);

  // draw cylinder
  markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  ignition::msgs::Set(markerMsg.mutable_scale(),
                      this->scaling);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(_x, _y,
                          this->height + this->scaling.Z() / 2.0,
                          0, 0, 0));
  markerMsg.set_id(_marker_id);
  bool result = node.Request("/marker", markerMsg);
  if (!result)
  {
    return false;
  }
  // draw direction cylinder
  markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  ignition::msgs::Set(markerMsg.mutable_scale(),
                      this->scaling);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(_x + cos(_yaw), _y + sin(_yaw),
                          this->height + this->scaling.Z() / 2.0,
                          0, M_PI/2, _yaw));
  markerMsg.set_id((_marker_id+1)*1000);
  result = node.Request("/marker", markerMsg);
  if (!result)
  {
    return false;
  }
  // draw text
  if (!_text.empty())
  {
    markerMsg.set_type(ignition::msgs::Marker::TEXT);
    markerMsg.set_text(_text);
    ignition::msgs::Set(markerMsg.mutable_scale(),
                        ignition::math::Vector3d(1.0, 1.0, 1.0));
    ignition::msgs::Set(markerMsg.mutable_pose(),
                        ignition::math::Pose3d(_x, _y - 0.2,
                            this->height + this->scaling.Z() + 0.8,
                            0, 0, 0));
    markerMsg.set_id((_marker_id + 1) * 10000);
    result = node.Request("/marker", markerMsg);
  }
  return result;

#else
  return false;
#endif
}
