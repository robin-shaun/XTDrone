/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_gimbal_controller_plugin.hh>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)

/* Keep these functions in the 'detail' namespace so that they
 * can be called from unit tests. */
namespace detail {
/////////////////////////////////////////////////
ignition::math::Vector3d ThreeAxisRot(
  double r11, double r12, double r21, double r31, double r32)
{
  return ignition::math::Vector3d(
    atan2( r31, r32 ),
    asin ( r21 ),
    atan2( r11, r12 ));
}

/////////////////////////////////////////////////
/// \TODO something to move into Angle class
/// \brief returns _angle1 normalized about
/// (_reference - M_PI, _reference + M_PI]
/// \param[in] _angle1 input angle
/// \param[in] _reference reference input angle for normalization
/// \return normalized _angle1 about _reference
double NormalizeAbout(double _angle, double reference)
{
  double diff = _angle - reference;
  // normalize diff about (-pi, pi], then add reference
  while (diff <= -M_PI)
  {
    diff += 2.0*M_PI;
  }
  while (diff > M_PI)
  {
    diff -= 2.0*M_PI;
  }
  return diff + reference;
}

/////////////////////////////////////////////////
/// \TODO something to move into Angle class
/// \brief returns shortest angular distance from _from to _to
/// \param[in] _from starting anglular position
/// \param[in] _to end angular position
/// \return distance traveled from starting to end angular positions
double ShortestAngularDistance(double _from, double _to)
{
  return NormalizeAbout(_to, _from) - _from;
}

/////////////////////////////////////////////////
ignition::math::Vector3d QtoZXY(
  const ignition::math::Quaterniond &_q)
{
  // taken from
  // http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
  // case zxy:
  ignition::math::Vector3d result = detail::ThreeAxisRot(
    -2*(_q.X()*_q.Y() - _q.W()*_q.Z()),
    _q.W()*_q.W() - _q.X()*_q.X() + _q.Y()*_q.Y() - _q.Z()*_q.Z(),
    2*(_q.Y()*_q.Z() + _q.W()*_q.X()),
    -2*(_q.X()*_q.Z() - _q.W()*_q.Y()),
    _q.W()*_q.W() - _q.X()*_q.X() - _q.Y()*_q.Y() + _q.Z()*_q.Z());
  return result;
}
}


/////////////////////////////////////////////////
GimbalControllerPlugin::GimbalControllerPlugin()
  :status("closed")
{
  /// defaults if sdf xml doesn't contain any pid gains
  this->pitchPid.Init(kPIDPitchP, kPIDPitchI, kPIDPitchD, kPIDPitchIMax, kPIDPitchIMin, kPIDPitchCmdMax, kPIDPitchCmdMin);
  this->rollPid.Init(kPIDRollP, kPIDRollI, kPIDRollD, kPIDRollIMax, kPIDRollIMin, kPIDRollCmdMax, kPIDRollCmdMin);
  this->yawPid.Init(kPIDYawP, kPIDYawI, kPIDYawD, kPIDYawIMax, kPIDYawIMin, kPIDYawCmdMax, kPIDYawCmdMin);
  this->pitchCommand = 0.5* M_PI;
  this->rollCommand = 0;
  this->yawCommand = 0;
  this->lastImuYaw = 0;
  this->rDir = kRollDir;
  this->pDir = kPitchDir;
  this->yDir = kYawDir;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->sdf = _sdf;

  // Create axis name -> pid map. It may be empty or not fully defined
  std::map<std::string, common::PID> pids_;

  if (_sdf->HasElement("control_gimbal_channels"))
  {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_gimbal_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");
    while(channel)
    {
      if (channel->HasElement("joint_axis"))
      {
        std::string joint_axis = channel->Get<std::string>("joint_axis");

        // setup joint control pid to control joint
        if (channel->HasElement("joint_control_pid"))
        {
          sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
          double p = 0;
          if (pid->HasElement("p"))
            p = pid->Get<double>("p");
          double i = 0;
          if (pid->HasElement("i"))
            i = pid->Get<double>("i");
          double d = 0;
          if (pid->HasElement("d"))
            d = pid->Get<double>("d");
          double iMax = 0;
          if (pid->HasElement("iMax"))
            iMax = pid->Get<double>("iMax");
          double iMin = 0;
          if (pid->HasElement("iMin"))
            iMin = pid->Get<double>("iMin");
          double cmdMax = 0;
          if (pid->HasElement("cmdMax"))
            cmdMax = pid->Get<double>("cmdMax");
          double cmdMin = 0;
          if (pid->HasElement("cmdMin"))
            cmdMin = pid->Get<double>("cmdMin");

          // insert pid gains into map for the respective named joint axis
          pids_.insert(std::pair<std::string, common::PID>(joint_axis, common::PID(p, i, d, iMax, iMin, cmdMax, cmdMin)));
        }
        channel = channel->GetNextElement("channel");
      }
    }
  }
  else
  {
    gzwarn << "Control channels for gimbal not found. Using default pid gains\n";
  }

  std::string yawJointName = "cgo3_vertical_arm_joint";
  this->yawJoint = this->model->GetJoint(yawJointName);
  if (this->sdf->HasElement("joint_yaw"))
  {
    // Add names to map
    yawJointName = sdf->Get<std::string>("joint_yaw");
    if (this->model->GetJoint(yawJointName))
    {
      this->yawJoint = this->model->GetJoint(yawJointName);

      // Try to find yaw rotation direction
      sdf::ElementPtr sdfElem = this->yawJoint->GetSDF();
      if(sdfElem->HasElement("axis"))
      {
        // Rotation is found
#if GAZEBO_MAJOR_VERSION >= 9
        yDir = this->yawJoint->LocalAxis(0)[2];
#else
        yDir = this->yawJoint->GetLocalAxis(0)[2];
#endif
      }
      else
      {
        // If user do not defines axis for yaw joint explicitly
        // then display warning
        gzwarn << "joint_yaw [" << yawJointName << "] axis do not defined?\n";
      }

      // Try to find respective pid for the named axis control
      std::map<std::string, common::PID>::iterator it = pids_.find("joint_yaw");
      if(it != pids_.end())
      {
        // Found pid for this axis (and therefore for this joint)
        this->yawPid = it->second;
      }
      else
      {
        // If user defines control channels for gimbal but don't define yaw gains explicitly
        // then display warning
        gzwarn << "joint_yaw [" << yawJointName << "] pid control gains do not defined?\n";
      }
    }
    else
    {
      gzwarn << "joint_yaw [" << yawJointName << "] does not exist?\n";
    }
  }
  if (!this->yawJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get yaw joint '"
          << yawJointName << "' " << endl;
  }

  std::string rollJointName = "cgo3_horizontal_arm_joint";
  this->rollJoint = this->model->GetJoint(rollJointName);
  if (this->sdf->HasElement("joint_roll"))
  {
    // Add names to map
    rollJointName = sdf->Get<std::string>("joint_roll");
    if (this->model->GetJoint(rollJointName))
    {
      this->rollJoint = this->model->GetJoint(rollJointName);

      // Try to find roll rotation direction
      sdf::ElementPtr sdfElem = this->rollJoint->GetSDF();
      if(sdfElem->HasElement("axis"))
      {
        // Rotation is found
#if GAZEBO_MAJOR_VERSION >= 9
        rDir = this->rollJoint->LocalAxis(0)[0];
#else
        rDir = this->rollJoint->GetLocalAxis(0)[0];
#endif
      }
      else
      {
        // If user do not defines axis for roll joint explicitly
        // then display warning
        gzwarn << "joint_roll [" << rollJointName << "] axis do not defined?\n";
      }

      // Try to find respective pid for the named axis control
      std::map<std::string, common::PID>::iterator it = pids_.find("joint_roll");
      if(it != pids_.end())
      {
        // Found pid for this axis (and therefore for this joint)
        this->rollPid = it->second;
      }
      else
      {
        // If user defines control channels for gimbal but don't define roll gains explicitly
        // then display warning
        gzwarn << "joint_roll [" << rollJointName << "] pid control gains do not defined?\n";
      }
    }
    else
    {
      gzwarn << "joint_roll [" << rollJointName << "] does not exist?\n";
    }
  }
  if (!this->rollJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get roll joint '"
          << rollJointName << "' " << endl;
  }

  std::string pitchJointName = "cgo3_camera_joint";
  this->pitchJoint = this->model->GetJoint(pitchJointName);
  if (this->sdf->HasElement("joint_pitch"))
  {
    // Add names to map
    pitchJointName = sdf->Get<std::string>("joint_pitch");
    if (this->model->GetJoint(pitchJointName))
    {
      this->pitchJoint = this->model->GetJoint(pitchJointName);

      // Try to find pitch rotation direction
      sdf::ElementPtr sdfElem = this->pitchJoint->GetSDF();
      if(sdfElem->HasElement("axis"))
      {
        // Rotation is found
#if GAZEBO_MAJOR_VERSION >= 9
        pDir = this->pitchJoint->LocalAxis(0)[1];
#else
        pDir = this->pitchJoint->GetLocalAxis(0)[1];
#endif
      }
      else
      {
        // If user do not defines axis for pitch joint explicitly
        // then display warning
        gzwarn << "joint_pitch [" << pitchJointName << "] axis do not defined?\n";
      }

      // Try to find respective pid for the named axis
      std::map<std::string, common::PID>::iterator it = pids_.find("joint_pitch");
      if(it != pids_.end())
      {
        // Found pid for this axis (and therefore for this joint)
        this->pitchPid = it->second;
      }
      else
      {
        // If user defines control channels for gimbal but don't define pitch gains explicitly
        // then display warning
        gzwarn << "joint_pitch [" << pitchJointName << "] pid control gains do not defined?\n";
      }
    }
    else
    {
      gzwarn << "joint_pitch [" << pitchJointName << "] does not exist?\n";
    }
  }
  if (!this->pitchJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get pitch joint '"
          << pitchJointName << "' " << endl;
  }

  // get imu sensors
  std::string cameraImuSensorName = "camera_imu";
  if (this->sdf->HasElement("gimbal_imu"))
  {
    // Add names to map
    cameraImuSensorName = sdf->Get<std::string>("gimbal_imu");
  }
#if GAZEBO_MAJOR_VERSION >= 7
  this->cameraImuSensor = std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(_model->SensorScopedName(cameraImuSensorName)[0]));
#elif GAZEBO_MAJOR_VERSION >= 6
  this->cameraImuSensor = boost::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(cameraImuSensorName));
#endif
  if (!this->cameraImuSensor)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get imu sensor '"
          << cameraImuSensorName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Init()
{
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 9
  this->node->Init(this->model->GetWorld()->Name());
  this->lastUpdateTime = this->model->GetWorld()->SimTime();
#else
  this->node->Init(this->model->GetWorld()->GetName());
  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
#endif

  // receive pitch command via gz transport
  std::string pitchTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_pitch_cmd";
  this->pitchSub = this->node->Subscribe(pitchTopic,
     &GimbalControllerPlugin::OnPitchStringMsg, this);
  // receive roll command via gz transport
  std::string rollTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_roll_cmd";
  this->rollSub = this->node->Subscribe(rollTopic,
     &GimbalControllerPlugin::OnRollStringMsg, this);
  // receive yaw command via gz transport
  std::string yawTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_yaw_cmd";
  this->yawSub = this->node->Subscribe(yawTopic,
     &GimbalControllerPlugin::OnYawStringMsg, this);

  // plugin update
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GimbalControllerPlugin::OnUpdate, this)));

  // publish pitch status via gz transport
  pitchTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_pitch_status";
  // Although gazebo above 7.4 support Any, still use GzString instead
  this->pitchPub = node->Advertise<gazebo::msgs::GzString>(pitchTopic);

  // publish roll status via gz transport
  rollTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_roll_status";
  // Although gazebo above 7.4 support Any, still use GzString instead
  this->rollPub = node->Advertise<gazebo::msgs::GzString>(rollTopic);

  // publish yaw status via gz transport
  yawTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_yaw_status";
  // Although gazebo above 7.4 support Any, still use GzString instead
  this->yawPub = node->Advertise<gazebo::msgs::GzString>(yawTopic);

  imuSub = node->Subscribe("~/" + model->GetName() + "/imu", &GimbalControllerPlugin::ImuCallback, this);

  gzmsg << "GimbalControllerPlugin::Init" << std::endl;
}

void GimbalControllerPlugin::ImuCallback(ImuPtr& imu_message)
{
  this->lastImuYaw = ignition::math::Quaterniond(imu_message->orientation().w(),
						 imu_message->orientation().x(),
						 imu_message->orientation().y(),
						 imu_message->orientation().z()).Euler()[2];
}

#if GAZEBO_MAJOR_VERSION > 7 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 4)
/// only gazebo 7.4 and above support Any
/////////////////////////////////////////////////
void GimbalControllerPlugin::OnPitchStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "pitch command received " << _msg->double_value() << std::endl;
  this->pitchCommand = _msg->double_value();
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnRollStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "roll command received " << _msg->double_value() << std::endl;
  this->rollCommand = _msg->double_value();
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnYawStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "yaw command received " << _msg->double_value() << std::endl;
  this->yawCommand = _msg->double_value();
}
#else
/////////////////////////////////////////////////
void GimbalControllerPlugin::OnPitchStringMsg(ConstGzStringPtr &_msg)
{
//  gzdbg << "pitch command received " << _msg->data() << std::endl;
  this->pitchCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnRollStringMsg(ConstGzStringPtr &_msg)
{
//  gzdbg << "roll command received " << _msg->data() << std::endl;
  this->rollCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnYawStringMsg(ConstGzStringPtr &_msg)
{
//  gzdbg << "yaw command received " << _msg->data() << std::endl;
  this->yawCommand = atof(_msg->data().c_str());
}
#endif

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnUpdate()
{
  if (!this->pitchJoint || !this->rollJoint || !this->yawJoint)
    return;

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time time = this->model->GetWorld()->SimTime();
#else
  common::Time time = this->model->GetWorld()->GetSimTime();
#endif
  if (time < this->lastUpdateTime)
  {
    gzerr << "time reset event\n";
    this->lastUpdateTime = time;
    return;
  }
  else if (time > this->lastUpdateTime)
  {
    double dt = (time - this->lastUpdateTime).Double();

    // We want yaw to control in body frame, not in global.
    this->yawCommand += this->lastImuYaw;

    // truncate command inside joint angle limits
#if GAZEBO_MAJOR_VERSION >= 9
    double rollLimited = ignition::math::clamp(this->rollCommand,
      rDir*this->rollJoint->UpperLimit(0),
	  rDir*this->rollJoint->LowerLimit(0));
    double pitchLimited = ignition::math::clamp(this->pitchCommand,
      pDir*this->pitchJoint->UpperLimit(0),
      pDir*this->pitchJoint->LowerLimit(0));
    double yawLimited = ignition::math::clamp(this->yawCommand,
      yDir*this->yawJoint->LowerLimit(0),
	  yDir*this->yawJoint->UpperLimit(0));
#else
    double rollLimited = ignition::math::clamp(this->rollCommand,
      rDir*this->rollJoint->GetUpperLimit(0).Radian(),
	  rDir*this->rollJoint->GetLowerLimit(0).Radian());
    double pitchLimited = ignition::math::clamp(this->pitchCommand,
      pDir*this->pitchJoint->GetUpperLimit(0).Radian(),
      pDir*this->pitchJoint->GetLowerLimit(0).Radian());
    double yawLimited = ignition::math::clamp(this->yawCommand,
      yDir*this->yawJoint->GetLowerLimit(0).Radian(),
	  yDir*this->yawJoint->GetUpperLimit(0).Radian());
#endif

    /// currentAngleYPRVariable is defined in roll-pitch-yaw-fixed-axis
    /// and gimbal is constructed using yaw-roll-pitch-variable-axis
    ignition::math::Vector3d currentAngleYPRVariable(
      this->cameraImuSensor->Orientation().Euler());

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d currentAnglePRYVariable(
      detail::QtoZXY(ignition::math::Quaterniond(currentAngleYPRVariable)));
#else
    ignition::math::Vector3d currentAnglePRYVariable(
      detail::QtoZXY(currentAngleYPRVariable));
#endif

    /// get joint limits (in sensor frame)
    /// TODO: move to Load() if limits do not change
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d lowerLimitsPRY
      (pDir*this->pitchJoint->LowerLimit(0),
       rDir*this->rollJoint->LowerLimit(0),
       yDir*this->yawJoint->LowerLimit(0));
    ignition::math::Vector3d upperLimitsPRY
      (pDir*this->pitchJoint->UpperLimit(0),
       rDir*this->rollJoint->UpperLimit(0),
       yDir*this->yawJoint->UpperLimit(0));
#else
    ignition::math::Vector3d lowerLimitsPRY
      (pDir*this->pitchJoint->GetLowerLimit(0).Radian(),
       rDir*this->rollJoint->GetLowerLimit(0).Radian(),
       yDir*this->yawJoint->GetLowerLimit(0).Radian());
    ignition::math::Vector3d upperLimitsPRY
      (pDir*this->pitchJoint->GetUpperLimit(0).Radian(),
       rDir*this->rollJoint->GetUpperLimit(0).Radian(),
       yDir*this->yawJoint->GetUpperLimit(0).Radian());
#endif

    // normalize errors
    double pitchError = detail::ShortestAngularDistance(
      pitchLimited, currentAnglePRYVariable.X());
    double rollError = detail::ShortestAngularDistance(
      rollLimited, currentAnglePRYVariable.Y());
    double yawError = detail::ShortestAngularDistance(
      yawLimited, currentAnglePRYVariable.Z());

    // Clamp errors based on current angle and estimated errors from rotations:
    // given error = current - target, then
    // if target (current angle - error) is outside joint limit, truncate error
    // so that current angle - error is within joint limit, i.e.:
    // lower limit < current angle - error < upper limit
    // or
    // current angle - lower limit > error > current angle - upper limit
    // re-expressed as clamps:
    // hardcoded negative joint axis for pitch and roll
    if (lowerLimitsPRY.X() < upperLimitsPRY.X())
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - upperLimitsPRY.X(),
        currentAnglePRYVariable.X() - lowerLimitsPRY.X());
    }
    else
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - lowerLimitsPRY.X(),
        currentAnglePRYVariable.X() - upperLimitsPRY.X());
    }
    if (lowerLimitsPRY.Y() < upperLimitsPRY.Y())
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y());
    }
    else
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y());
    }
    if (lowerLimitsPRY.Z() < upperLimitsPRY.Z())
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z());
    }
    else
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z());
    }

    // apply forces to move gimbal
    double pitchForce = this->pitchPid.Update(pitchError, dt);
    this->pitchJoint->SetForce(0, pDir*pitchForce);

    double rollForce = this->rollPid.Update(rollError, dt);
    this->rollJoint->SetForce(0, rDir*rollForce);

    double yawForce = this->yawPid.Update(yawError, dt);
    this->yawJoint->SetForce(0, yDir*yawForce);

    // ignition::math::Vector3d angles = this->imuSensor->Orientation().Euler();
    // gzerr << "ang[" << angles.X() << ", " << angles.Y() << ", " << angles.Z()
    //       << "] cmd[ " << this->rollCommand
    //       << ", " << this->pitchCommand << ", " << this->yawCommand
    //       << "] err[ " << rollError
    //       << ", " << pitchError << ", " << yawError
    //       << "] frc[ " << rollForce
    //       << ", " << pitchForce << ", " << yawForce << "]\n";


    this->lastUpdateTime = time;
  }

  static int i =1000;
  if (++i>100)
  {
    i = 0;
 // There is bug when use gazebo::msgs::Any m, so still use GzString instead
 // Although gazebo above 7.4 support Any, still use GzString instead
    std::stringstream ss;
    gazebo::msgs::GzString m;
#if GAZEBO_MAJOR_VERSION >= 9
    ss << this->pitchJoint->Position(0);
    m.set_data(ss.str());
    this->pitchPub->Publish(m);

    ss << this->rollJoint->Position(0);
    m.set_data(ss.str());
    this->rollPub->Publish(m);

    ss << this->yawJoint->Position(0);
    m.set_data(ss.str());
    this->yawPub->Publish(m);
#else
    ss << this->pitchJoint->GetAngle(0).Radian();
    m.set_data(ss.str());
    this->pitchPub->Publish(m);

    ss << this->rollJoint->GetAngle(0).Radian();
    m.set_data(ss.str());
    this->rollPub->Publish(m);

    ss << this->yawJoint->GetAngle(0).Radian();
    m.set_data(ss.str());
    this->yawPub->Publish(m);
#endif
  }
}

