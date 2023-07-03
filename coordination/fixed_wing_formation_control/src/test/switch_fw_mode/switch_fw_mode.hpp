/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#ifndef _SWITCH_FW_MODE_HPP_
#define _SWITCH_FW_MODE_HPP_

#include "../../fixed_wing_lib/syslib.hpp"
#include "fixed_wing_formation_control/Fw_cmd_mode.h"
#include <iostream>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
// //为主机切换为OFFBOARD添加的
// #include<mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#define SWITCH_FW_MODE_INFO(a) cout << "[SWUTCH_FW_MODE_INFO]:" << a << endl

using namespace std;

class SWITCH_FW_MODE {
public:
  void set_planeID(int id);
  void run();

private:
  int planeID{1};
  string uavID{"uav1/"};

  ros::NodeHandle nh;
  ros::Time begin_time;          /* 记录启控时间 */

  /* 无人机当前状态[包含上锁状态 模式] (从飞控中读取) */
  mavros_msgs::State current_state;
  mavros_msgs::SetMode mode_cmd; /*  模式容器 */
  fixed_wing_formation_control::Fw_cmd_mode task_cmd_mode; /* 任务模式切换 */

  // mavros_msgs::PositionTarget position_target;

  int times_out = 100;  /* 次数限制 */
  int counters = 0;     /* 计数器 */
  bool outflag = false; /* 退出标志位 */

  float get_ros_time(ros::Time begin);

  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
  }
};
#endif
