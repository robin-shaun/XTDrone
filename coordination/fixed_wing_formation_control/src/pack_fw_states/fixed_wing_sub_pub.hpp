/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

//***话题的设置有冗余，可根据自身需要选择

#ifndef _FIXED_WING_SUB_PUB_HPP_
#define _FIXED_WING_SUB_PUB_HPP_

// ros程序必备头文件
#include <ros/ros.h>
//mavros相关头文件
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h> //提示local——position在这里
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/VFR_HUD.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h> //GPS Fix.
#include <std_msgs/Float64.h>
#include <sensor_msgs/BatteryState.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h> //UTM coords
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h> //Velocity fused by FCU
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

//setpoint_raw:在这个底下，全都有，SET_ATTITUDE_TARGET++SET_POSITION_TARGET_LOCAL_NED
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include "fixed_wing_formation_control/FWstates.h" //自定义的飞机的状态消息
#include "fixed_wing_formation_control/FWcmd.h"

#include "../fixed_wing_lib/mathlib.hpp"

#ifndef the_space_between_lines
#define the_space_between_lines 1  //为了打印中间空格
#define the_space_between_blocks 3 //为了打印中间空格
#endif

class _FIXED_WING_SUB_PUB
{

private:
public:
    //订阅的数据暂时容器
    mavros_msgs::State current_state; //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

    sensor_msgs::Imu imu;

    sensor_msgs::NavSatFix global_position_form_px4;

    std_msgs::Float64 global_rel_alt_from_px4;

    nav_msgs::Odometry umt_position_from_px4;

    nav_msgs::Odometry ugv_position_from_gazebo;

    geometry_msgs::TwistStamped velocity_global_fused_from_px4;

    geometry_msgs::TwistStamped velocity_ned_fused_from_px4;

    geometry_msgs::PoseStamped local_position_from_px4;

    geometry_msgs::AccelWithCovarianceStamped acc_ned_from_px4;

    geometry_msgs::TwistWithCovarianceStamped wind_estimate_from_px4;

    sensor_msgs::BatteryState battrey_state_from_px4;

    mavros_msgs::WaypointList waypoint_list;

    mavros_msgs::WaypointReached waypoint_reached;

    mavros_msgs::Altitude altitude_from_px4;

    mavros_msgs::VFR_HUD air_ground_speed_from_px4;

    fixed_wing_formation_control::FWcmd cmd_from_controller; //来自各种控制器的四通道控制量

    //服务项暂存容器
    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arming_service;

    mavros_msgs::WaypointSetCurrent waypoint_set_current_service;

    mavros_msgs::WaypointPull waypoint_pull_service;

    mavros_msgs::WaypointPush waypoint_push_service;

    mavros_msgs::WaypointClear waypoint_clear_service;

    //发布的数据暂时容器
    mavros_msgs::PositionTarget local_pos_sp;

    mavros_msgs::GlobalPositionTarget global_pos_sp;

    mavros_msgs::AttitudeTarget att_sp;

    fixed_wing_formation_control::FWstates fw_states_form_mavros; //这个是自定义的完整的飞机状态消息

    float att_sp_Euler[3];
    float thrust_sp;

    float PIX_Euler_target[3]; //无人机 期望欧拉角(从飞控中读取)
    float att_angle_Euler[3];  //无人机当前欧拉角(从飞控中读取)  ***imu四元数，在本头文件中进行了转换

    float get_ros_time(ros::Time begin) //获取ros时间
    {
        ros::Time time_now = ros::Time::now();
        float currTimeSec = time_now.sec - begin.sec;
        float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
        return (currTimeSec + currTimenSec);
    }

    void state_cb(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state = *msg;
    }

    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        imu = *msg;
        float q[4];
        q[0] = msg->orientation.w;
        q[1] = msg->orientation.x;
        q[2] = msg->orientation.y;
        q[3] = msg->orientation.z;

        quaternion_2_euler(q, att_angle_Euler);
    }

    void global_position_form_px4_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        global_position_form_px4 = *msg;
    }

    void fixed_wing_global_rel_alt_from_px4_cb(const std_msgs::Float64::ConstPtr &msg)
    {
        global_rel_alt_from_px4 = *msg;
    }

    void umt_position_from_px4_cb(const nav_msgs::Odometry::ConstPtr &msg)
    {
        umt_position_from_px4 = *msg;
    }

    void velocity_global_fused_from_px4_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        velocity_global_fused_from_px4 = *msg;
    }

    void velocity_ned_fused_from_px4_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        velocity_ned_fused_from_px4 = *msg;
    }

    void local_position_from_px4_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        local_position_from_px4 = *msg;
    }

    void acc_ned_from_px4_cb(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr &msg)
    {
        acc_ned_from_px4 = *msg;
    }

    void wind_estimate_from_px4_cb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
    {
        wind_estimate_from_px4 = *msg;
    }

    void battrey_state_from_px4_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
    {
        battrey_state_from_px4 = *msg;
    }

    void waypoints_reached_from_px4_cb(const mavros_msgs::WaypointReached::ConstPtr &msg)
    {
        waypoint_reached = *msg;
    }

    void waypointlist_from_px4_cb(const mavros_msgs::WaypointList::ConstPtr &msg)
    {
        waypoint_list = *msg;
    }

    void altitude_from_px4_cb(const mavros_msgs::Altitude::ConstPtr &msg)
    {
        altitude_from_px4 = *msg;
    }

    void air_ground_speed_from_px4_cb(const mavros_msgs::VFR_HUD::ConstPtr &msg)
    {
        air_ground_speed_from_px4 = *msg;
    }

    void cmd_from_controller_cb(const fixed_wing_formation_control::FWcmd::ConstPtr &msg)
    {
        cmd_from_controller = *msg;
    }

    void ugv_position_from_gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ugv_position_from_gazebo = *msg;
    }
};

#endif