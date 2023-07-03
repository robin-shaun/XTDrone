/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */
//***建议参数的读取放在外部文件中，这样便于修改，不过这里直接结构体赋值了

#ifndef _TASK_MAIN_HPP_
#define _TASK_MAIN_HPP_

#include <ros/ros.h>
#include <iostream>
#include "../fixed_wing_lib/syslib.hpp"
#include "fixed_wing_formation_control/FWstates.h"
#include "fixed_wing_formation_control/FWcmd.h"
#include "fixed_wing_formation_control/Leaderstates.h"
#include "fixed_wing_formation_control/Formation_control_states.h"
#include "fixed_wing_formation_control/Fwmonitor.h"
#include "fixed_wing_formation_control/Fw_cmd_mode.h"
#include "fixed_wing_formation_control/Fw_current_mode.h"
#include "../formation_controller/formation_controller.hpp"
#include "../formation_controller/abs_formation_controller.hpp"

using namespace std;

#define TASK_MAIN_INFO(a) cout << "[TASK_MAIN_INFO]:" << a << endl

class TASK_MAIN
{
private:
    int planeID{1};                                                   /*飞机编号*/

    int formation_type_id{0};    //***队形切换

    string uavID{ "uav1/"}; /* 本机编号，用于命名空间 */

    string leaderID{ "uav0/"};/* 领机编号，用于命名空间 */

    void print_data(const struct ABS_FORMATION_CONTROLLER ::_s_fw_states *p); /*打印数据*/

    ros::NodeHandle nh; /*ros句柄*/
    ros::Time begin_time;
    float current_time;
    float get_ros_time(ros::Time begin); /*获取当前时间*/

    ros::Subscriber fwmonitor_sub;               /*【订阅】来自fw_monitor任务状态的flags*/
    ros::Subscriber fw_states_sub;               /*【订阅】来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber leader_states_sub;           /*【订阅】来自通讯节点，或者视觉节点领机全部状态量*/
    ros::Subscriber fw_cmd_mode_sub;             /*【订阅】来自commander状态机的控制指令*/
    ros::Publisher formation_control_states_pub; /*【发布】编队控制状态量*/
    ros::Publisher fw_cmd_pub;                   /*【发布】飞机四通道控制量*/
    ros::Publisher fw_current_mode_pub;          /*【发布】飞机当前处于的任务状态*/

    fixed_wing_formation_control::FWstates fwstates;                                 /*自定义--飞机打包的全部状态*/
    fixed_wing_formation_control::FWcmd fw_4cmd;                                     /*自定义--飞机四通道控制量*/
    fixed_wing_formation_control::Leaderstates leaderstates;                         /*自定义--领机状态*/
    fixed_wing_formation_control::Formation_control_states formation_control_states; /*自定义--编队控制状态量*/
    fixed_wing_formation_control::Fwmonitor fwmonitor_flag;                          /*自定义--任务状态的flags*/
    fixed_wing_formation_control::Fw_cmd_mode fw_cmd_mode;                           /*自定义--来自commander的命令飞机模式*/
    fixed_wing_formation_control::Fw_current_mode fw_current_mode;                   /*自定义--要发布的飞机当前任务模式*/

    void ros_sub_pub();                                                                     /*ros消息订阅与发布*/
    void fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg);     /*任务状态flagscallback*/
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg);          /*本机状态callback*/
    void leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg); /*领机状态callback*/
    void fw_cmd_mode_cb(const fixed_wing_formation_control::Fw_cmd_mode::ConstPtr &msg);    /*commander指令callback*/

    void control_formation();               /*编队控制主函数*/
    ABS_FORMATION_CONTROLLER formation_controller; /*编队控制器*/
    string fw_col_mode_current{"MANUAL"};   /*当前模式*/
    string fw_col_mode_last{"MANUAL"};      /*上一时刻模式*/

    struct ABS_FORMATION_CONTROLLER::_s_fw_model_params fw_params;                   /*飞机模型参数*/
    struct ABS_FORMATION_CONTROLLER::_s_tecs_params tecs_params;                     /*编队控制器内部TECS控制器参数*/

    struct ABS_FORMATION_CONTROLLER::_s_leader_states leader_states;       /*领机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states thisfw_states;           /*本机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_4cmd formation_cmd;                /*四通道控制量*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_error formation_error;          /*编队误差以及偏差*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_sp formation_sp;                /*编队控制运动学期望值*/
    void formation_states_pub();                                    /*发布编队控制器控制状态*/

public:
  void run();
  void set_planeID(int id);
};

#endif
