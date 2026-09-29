/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#ifndef _PACK_FW_STATES_HPP_
#define _PACK_FW_STATES_HPP_

#include <ros/ros.h>                                /* 添加ROSC++客户端库必须包含的头文件 */
#include <iostream>                                 /* 添加输入输出流文件	*/
#include "fixed_wing_formation_control/FWstates.h"  /* 固定翼的编队控制，自定义可能用到的消息类型 */
#include "fixed_wing_formation_control/FWcmd.h"     /* 固定翼控制指令的期望值文件 */
#include "fixed_wing_sub_pub.hpp"                   /* ROS回调函数处理头文件 */
#include "../fixed_wing_lib/syslib.hpp"             /* 添加时间戳文件	*/

/* 定义打印输出语句 */
#define PACK_FW_STATES_INFO(a) cout<<"[PACK_FW_STATES_INFO]:"<<a<<endl
/* 命名空间：定义标识符的各种可见范围*/
using namespace std;

class PACK_FW_STATES
{

private:
    int planeID;   /* 当前无人机id号 */
    int numUAV{6};    /* 无人机总数为6 */
    int numLeader{3}; /* 长机数目为3 */

    string uavID{ "uav0/"};

    int print_counter{0};
    /* ROS回调函数处理头文件在本文件中命名为fixed_wing_sub_pub */
    _FIXED_WING_SUB_PUB fixed_wing_sub_pub;

    ros::NodeHandle nh;/*ros句柄，nh在<node_name>下*/

    /* 创建client对象，用于存储订阅的信息 */
    ros::ServiceClient set_mode_client; /* 创建client对象，用于存储Set_Mode订阅的信息 */
    ros::ServiceClient arming_client;
    ros::ServiceClient waypoint_setcurrent_client;
    ros::ServiceClient waypoint_pull_client;
    ros::ServiceClient waypoint_push_client;
    ros::ServiceClient waypoint_clear_client;
    /* 创建订阅/发布的对象，用于存储订阅的信息 */
    ros::Publisher fixed_wing_local_pos_sp_pub;
    ros::Publisher fixed_wing_global_pos_sp_pub;
    ros::Publisher fixed_wing_local_att_sp_pub;
    ros::Publisher fixed_wing_states_pub; //发布打包的飞机信息

    ros::Subscriber // 【订阅对象】无人机ned三向加速度
        fixed_wing_battrey_state_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机ned三向加速度
        fixed_wing_wind_estimate_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机ned三向加速度
        fixed_wing_acc_ned_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机ned三向速度
        fixed_wing_velocity_ned_fused_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机ned位置
        fixed_wing_local_position_from_px4;

    ros::Subscriber // 【订阅对象】无人机gps三向速度
        fixed_wing_velocity_global_fused_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机ump位置
        fixed_wing_umt_position_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机gps相对alt
        fixed_wing_global_rel_alt_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机gps位置
        fixed_wing_global_position_form_px4_sub;

    ros::Subscriber // 【订阅对象】无人机imu信息，
        fixed_wing_imu_sub;

    ros::Subscriber // 【订阅对象】无人机当前模式
        fixed_wing_states_sub;

    ros::Subscriber // 【订阅对象】无人机当前航点
        fixed_wing_waypoints_sub;

    ros::Subscriber // 【订阅对象】无人机到达的航点
        fixed_wing_waypointsreach_sub;

    ros::Subscriber // 【订阅对象】无人机的高度
        fixed_wing_altitude_from_px4_sub;

    ros::Subscriber // 【订阅对象】无人机的空速地速
        fixed_wing_air_ground_speed_from_px4_sub;

    ros::Subscriber // 【订阅对象】来自上层控制器的四通道控制量
        fixed_wing_cmd_from_controller_sub;

    ros::Subscriber // 【订阅对象】订阅ugv位置
        ugv_position_from_gazebo_sub;

    float att_angle[3], att_quat[4]; //转换四元数中间量

    /* 子函数（组） */

    /* 把需要发送给PX4的控制量发送给MAVROS */
    void msg_to_mavros();
    /* 把切换模式的服务发送给MAVROS */
    void srv_to_mavros();
    /* ros的订阅发布声明函数 */
    void ros_sub_and_pub();
    /*把订阅的所有消息组成包，发布出去*/
    void pack_fw_states();

   // void leader_fol_ugv();   //***用来实现领机跟随ugv运动 位置控制

   // void srv_to_mavros_leader();  //用来切换领机的模式

public:

    /* 主循环，在main中调用 */
    void run(int argc, char **argv);
    /* 设定当前飞机的ID */
    void set_planeID(int id);
    /* 设定初始虚拟目标点弧长 */
    void set_scurve(float scurve);
};
#endif
