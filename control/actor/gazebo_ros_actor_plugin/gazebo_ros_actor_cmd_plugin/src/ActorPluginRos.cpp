/*
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
/**
 *  \author David Leins
 *  \date 28th of November 2018
 *  \desc Gazebo ros plugin to steer an actor with twist messages over ros. Based on actor_plugin. 
 */

#include "actor_plugin_ros/ActorPluginRos.hpp"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPluginRos)

#define WALKING_ANIMATION "walking"
#define WAVING_ANIMATION "waving"

/////////////////////////////////////////////////
ActorPluginRos::ActorPluginRos()
{
}

/////////////////////////////////////////////////
void ActorPluginRos::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  gazebo_ros_ = GazeboRosPtr(new GazeboRos(_model, _sdf, "ActorRos"));
  gazebo_ros_->isInitialized();

  // cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe("cmd_vel", 1, &ActorPluginRos::NewVelCmdCallback, this);
  // ROS_INFO_NAMED("ActorRos", "%s: Subscribe to %s", gazebo_ros_->info(), cmd_vel_subscriber_.getTopic().c_str());

  cmd_pose_subscriber_ = gazebo_ros_->node()->subscribe("cmd_pose", 1, &ActorPluginRos::CmdPoseCallback, this);
  ROS_INFO_NAMED("ActorRos", "%s: Subscribe to %s", gazebo_ros_->info(), cmd_pose_subscriber_.getTopic().c_str());

  wave_toggle_service_ = gazebo_ros_->node()->advertiseService("actor_toggle_wave", &ActorPluginRos::ToggleWaveAnimation, this);
  ROS_INFO_NAMED("ActorRos", "%s: Advertise service %s", gazebo_ros_->info(), wave_toggle_service_.getService().c_str());
  this->wave_toggled = false;

  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&ActorPluginRos::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  if (_sdf->HasElement("init_pose"))
    this->init_pose = _sdf->Get<ignition::math::Pose3d>("init_pose");
  else
    this->init_pose = ignition::math::Pose3d(0, 0, 1.0191, 1.57, 0, 0);

  // this->velocity =20;
  // Make sure the actor stays within bounds
  this->init_pose.Pos().X(std::max(-50.0, std::min(100.0, this->init_pose.Pos().X())));
  this->init_pose.Pos().Y(std::max(-50.0, std::min(50.0, this->init_pose.Pos().Y())));
  this->init_pose.Pos().Z(1.0191);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (this->init_pose.Pos() -
                             this->actor->WorldPose().Pos())
                                .Length();

  this->actor->SetWorldPose(this->init_pose, false, false);
  // this->actor->SetScriptTime(this->actor->ScriptTime() +
  //                            (distanceTraveled * 5.0));
}

/////////////////////////////////////////////////
void ActorPluginRos::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  // this->target = ignition::math::Vector3d(0, 0, 1.2138);
  this->target = this->init_pose.Pos();

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;
    this->last_linear = ignition::math::Vector3d(0, 0, 0);
    this->last_angle = 0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
    this->wave_toggled = false;
  }
}

/////////////////////////////////////////////////
void ActorPluginRos::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-50, 100));
    newTarget.Y(ignition::math::Rand::DblUniform(-50, 50));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos() - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
void ActorPluginRos::CmdPoseCallback(const geometry_msgs::Point::ConstPtr &cmd_msg)
{
  GET_CMD_FLAG = true;
  target[0] = cmd_msg->x;
  target[1] = cmd_msg->y;
  target[2] = cmd_msg->z;
  //target[2] is cmd_vel
  //target = ignition::math::Vector3d(10, 10, 1.0191);
  //std::cout << "I'm here!" << endl;
}

/////////////////////////////////////////////////
// void ActorPluginRos::NewVelCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
//   double angle = this->actor->WorldPose().Rot().Yaw() * -1;
//   this->last_linear = ignition::math::Vector3d(
//     cmd_msg->linear.y * cos(angle) - cmd_msg->linear.x * sin(angle),
//     -(cmd_msg->linear.y * sin(angle) + cmd_msg->linear.x * cos(angle)),
//     0
//   );
//   this->last_angle = cmd_msg->angular.z;
// }

/////////////////////////////////////////////////
bool ActorPluginRos::ToggleWaveAnimation(ros_actor_cmd_pose_plugin_msgs::ToggleActorWaving::Request &req, ros_actor_cmd_pose_plugin_msgs::ToggleActorWaving::Response &res)
{
  ROS_INFO("request: toggle waving animation");

  this->velocity = 0.8;
  this->lastUpdate = 0;

  this->target = ignition::math::Vector3d(0, 0, 1.2138);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WAVING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WAVING_ANIMATION << " not found.\n";
    res.exit = false;
    this->wave_toggled = false;
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WAVING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;
    this->last_linear = ignition::math::Vector3d(0, 0, 0);
    this->last_angle = 0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);
    this->wave_toggled = true;
    res.exit = true;
  }

  ROS_INFO("sending back response: %s", res.exit ? "true" : "false");

  return true;
}

/////////////////////////////////////////////////
void ActorPluginRos::OnUpdate(const common::UpdateInfo &_info)
{
  this->velocity = target[2];
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  if (this->wave_toggled)
  {
    this->actor->SetScriptTime(this->actor->ScriptTime() +
                               dt);
  }
  else
  {

    ignition::math::Pose3d pose = this->actor->WorldPose();
    ignition::math::Vector3d pos = this->target - pose.Pos();
    ignition::math::Vector3d rpy = pose.Rot().Euler();

    double distance = pos.Length();

    // Choose a new target position if the actor has reached its current target.
    if (distance < 0.01)
    {
      // FIXME: Commented out to prevent swerve after actor reached its target.
      //this->ChooseNewTarget();
      pos = this->target - pose.Pos();
    }
    // Choose a suitable velocity at different distance.
    if (distance > 1.0)
    {
      this->velocity = velocity / distance;
    }
    else if (distance <= 1.0)
    {
      this->velocity = velocity;
    }

    // Compute the yaw orientation.
    ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
    yaw.Normalize();

    //ignition::math::Angle yaw = this->last_angle;

    // Rotate in place, instead of jumping.
    if (std::abs(yaw.Radian()) > IGN_DTOR(10))
    {
      pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian() * 0.1);
    }
    else
    {
      pose.Pos() += pos * this->velocity * dt;
      pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian());
    }

    // Make sure the actor stays within bounds
    pose.Pos().X(std::max(-50.0, std::min(100.0, pose.Pos().X())));
    pose.Pos().Y(std::max(-50.0, std::min(50.0, pose.Pos().Y())));
    pose.Pos().Z(1.0191);

    // Distance traveled is used to coordinate motion with the walking
    // animation
    double distanceTraveled = (pose.Pos() -
                               this->actor->WorldPose().Pos())
                                  .Length();
    if (GET_CMD_FLAG == false)
    {
      this->actor->SetWorldPose(this->init_pose, false, false);
    }
    else
    {
      this->actor->SetWorldPose(pose, false, false);
    }
     
    this->actor->SetScriptTime(this->actor->ScriptTime() +
                               (distanceTraveled * 5.0));
    this->lastUpdate = _info.simTime;

    gzdbg << "[XTDrone_Actor_Plugin]: Publish topic actor_pose_pub" << std::endl;
    gzdbg << "Target:  x:" << target[0] << ", y:" << target[1] << ",vel:" << target[2] << std::endl;
    gzdbg << "Actor_Position:  " << std::dec << pose.Pos().X() << "," << pose.Pos().Y() << "," << pose.Pos().Z() << std::endl;
    gzdbg << "init_pose:  " << std::dec << init_pose << "vel: " << this->velocity << std::endl;
  }
}
