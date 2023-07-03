/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#ifndef _PACK_FW_STATES_HPP_
#define _PACK_FW_STATES_HPP_

#include <ros/ros.h>
#include <iostream>
#include "fixed_wing_formation_control/FWstates.h" /*自定义的飞机的状态消息*/
#include "fixed_wing_formation_control/FWcmd.h"
#include "fixed_wing_sub_pub.hpp"
#include "../fixed_wing_lib/syslib.hpp"

#define PACK_FW_STATES_INFO(a) cout<<"[PACK_FW_STATES_INFO]:"<<a<<endl
using namespace std;

class PACK_FW_STATES
{

private:
    int planeID{1};

    string uavID{ "uav1/"};

    int print_counter{0};

    _FIXED_WING_SUB_PUB fixed_wing_sub_pub;

    ros::NodeHandle nh;

    ros::ServiceClient set_mode_client;

    ros::ServiceClient arming_client;

    ros::ServiceClient waypoint_setcurrent_client;

    ros::ServiceClient waypoint_pull_client;

    ros::ServiceClient waypoint_push_client;

    ros::ServiceClient waypoint_clear_client;

    ros::Publisher fixed_wing_local_pos_sp_pub;

    ros::Publisher fixed_wing_global_pos_sp_pub;

    ros::Publisher fixed_wing_local_att_sp_pub;

    ros::Publisher fixed_wing_states_pub; //发布打包的飞机信息

    ros::Subscriber // 【订阅】无人机ned三向加速度
        fixed_wing_battrey_state_from_px4_sub;

    ros::Subscriber // 【订阅】无人机ned三向加速度
        fixed_wing_wind_estimate_from_px4_sub;

    ros::Subscriber // 【订阅】无人机ned三向加速度
        fixed_wing_acc_ned_from_px4_sub;

    ros::Subscriber // 【订阅】无人机ned三向速度
        fixed_wing_velocity_ned_fused_from_px4_sub;

    ros::Subscriber // 【订阅】无人机ned位置
        fixed_wing_local_position_from_px4;

    ros::Subscriber // 【订阅】无人机gps三向速度
        fixed_wing_velocity_global_fused_from_px4_sub;

    ros::Subscriber // 【订阅】无人机ump位置
        fixed_wing_umt_position_from_px4_sub;

    ros::Subscriber //【订阅】无人机gps相对alt
        fixed_wing_global_rel_alt_from_px4_sub;

    ros::Subscriber // 【订阅】无人机gps位置
        fixed_wing_global_position_form_px4_sub;

    ros::Subscriber // 【订阅】无人机imu信息，
        fixed_wing_imu_sub;

    ros::Subscriber // 【订阅】无人机当前模式
        fixed_wing_states_sub;

    ros::Subscriber // 【订阅】无人机当前航点
        fixed_wing_waypoints_sub;

    ros::Subscriber // 【订阅】无人机到达的航点
        fixed_wing_waypointsreach_sub;

    ros::Subscriber // 【订阅】无人机的高度
        fixed_wing_altitude_from_px4_sub;

    ros::Subscriber // 【订阅】无人机的空速地速
        fixed_wing_air_ground_speed_from_px4_sub;

    ros::Subscriber //{订阅}来自上层控制器的四通道控制量
        fixed_wing_cmd_from_controller_sub;

    ros::Subscriber //***订阅ugv位置
        ugv_position_from_gazebo_sub;

    float att_angle[3], att_quat[4]; //转换四元数中间量

    /* 子函数（组） */
    void msg_to_mavros();

    void srv_to_mavros();

    void ros_sub_and_pub();

    void pack_fw_states();

    void leader_fol_ugv();   //***用来实现领机跟随ugv运动 位置控制

    void srv_to_mavros_leader();  //***用来切换领机的模式

public:

    void run(int argc, char **argv);

    void set_planeID(int id);
};
#endif
