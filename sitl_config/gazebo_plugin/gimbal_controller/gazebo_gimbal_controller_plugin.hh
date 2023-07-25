/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/* Desc: A basic gimbal controller
 * Author: John Hsu
 */

#ifndef _GAZEBO_GIMBAL_CONTROLLER_PLUGIN_HH_
#define _GAZEBO_GIMBAL_CONTROLLER_PLUGIN_HH_

#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <optional>
#include <thread>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>
#include <development/mavlink.h>

namespace gazebo
{
  // Default PID gains
  static double kPIDPitchP = 5.0;
  static double kPIDPitchI = 0.0;
  static double kPIDPitchD = 0.0;
  static double kPIDPitchIMax = 0.0;
  static double kPIDPitchIMin = 0.0;
  static double kPIDPitchCmdMax = 0.3;
  static double kPIDPitchCmdMin = -0.3;

  static double kPIDRollP = 5.0;
  static double kPIDRollI = 0.0;
  static double kPIDRollD = 0.0;
  static double kPIDRollIMax = 0.0;
  static double kPIDRollIMin = 0.0;
  static double kPIDRollCmdMax = 0.3;
  static double kPIDRollCmdMin = -0.3;

  static double kPIDYawP = 1.0;
  static double kPIDYawI = 0.0;
  static double kPIDYawD = 0.0;
  static double kPIDYawIMax = 0.0;
  static double kPIDYawIMin = 0.0;
  static double kPIDYawCmdMax = 1.0;
  static double kPIDYawCmdMin = -1.0;

  // Default rotation directions
  static double kRollDir = -1.0;
  static double kPitchDir = -1.0;
  static double kYawDir = 1.0;

  class GAZEBO_VISIBLE GimbalControllerPlugin : public ModelPlugin
  {
    public: GimbalControllerPlugin();
    public: ~GimbalControllerPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: virtual void Init();

    private: void OnUpdate();

    private: bool InitUdp();
    private: void SendHeartbeat();
    private: void SendGimbalDeviceInformation();
    private: void SendGimbalDeviceAttitudeStatus();
    private: void SendResult(uint8_t target_sysid, uint8_t target_compid, uint16_t command, MAV_RESULT result);
    private: void SendMavlinkMessage(const mavlink_message_t& msg);
    private: void RxThread();
    private: void HandleMessage(const mavlink_message_t& msg);
    private: void HandleCommandLong(const mavlink_message_t& msg);
    private: void HandleGimbalDeviceSetAttitude(const mavlink_message_t& msg);
    private: void HandleAutopilotStateForGimbalDevice(const mavlink_message_t& msg);
    private: void HandleRequestMessage(uint8_t target_sysid, uint8_t target_compid, const mavlink_command_long_t& command_long);
    private: void HandleSetMessageInterval(uint8_t target_sysid, uint8_t target_compid, const mavlink_command_long_t& command_long);

    private: static std::optional<double> calcSetpoint(double dt, double lastSetpoint, double newSetpoint, double newRateSetpoint);

    private: std::mutex cmd_mutex;

    private: sdf::ElementPtr sdf;

    private: std::vector<event::ConnectionPtr> connections;

    private: physics::ModelPtr model;

    /// \brief yaw camera
    private: physics::JointPtr yawJoint;

    /// \brief camera roll joint
    private: physics::JointPtr rollJoint;

    /// \brief camera pitch joint
    private: physics::JointPtr pitchJoint;

    private: sensors::ImuSensorPtr cameraImuSensor;
    private: double vehicleYawRad {0.0};
    private: std::string status;

    private: double rDir;
    private: double pDir;
    private: double yDir;

    private: std::mutex setpointMutex {};
    private: double lastRollSetpoint {0.0};
    private: double lastPitchSetpoint {0.0};
    private: double lastYawSetpoint {0.0};
    private: double rollSetpoint {0.0};
    private: double pitchSetpoint {0.0};
    private: double yawSetpoint {0.0};
    private: bool yawLock {false};
    private: double rollRateSetpoint {NAN};
    private: double pitchRateSetpoint {NAN};
    private: double yawRateSetpoint {NAN};

    private: transport::NodePtr node;

    private: common::PID pitchPid;
    private: common::PID rollPid;
    private: common::PID yawPid;
    private: common::Time lastUpdateTime;

    private: std::unique_ptr<std::thread> rxThread {};
    private: std::atomic<bool> shouldExit {false};
    private: int sock;
    private: struct sockaddr_in myaddr;
    private: common::Time lastHeartbeatSentTime;
    private: const double heartbeatIntervalS {1.0};
    private: common::Time lastAttitudeStatusSentTime;

    private: const double defaultAttitudeStatusIntervalS {0.1};
    private: double attitudeStatusIntervalS {defaultAttitudeStatusIntervalS};
    private: bool sendingAttitudeStatus {true};

    private: static constexpr uint8_t ourSysid {1};
    private: static constexpr uint8_t ourCompid {MAV_COMP_ID_GIMBAL};
    private: static constexpr uint8_t mavlinkChannel {MAVLINK_COMM_3};

    private: std::string udp_gimbal_host_ip;
    private: int udp_gimbal_port_remote;
  };
}
#endif
