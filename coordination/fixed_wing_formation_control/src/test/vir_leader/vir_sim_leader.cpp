/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#include "vir_sim_leader.hpp"

/**
 * @Input: int
 * @Output: 
 * @Description: 设定当前飞机的ID
 */
void VIR_SIM_LEADER::set_planeID(int id) {
  planeID = id;
  switch (planeID) {
  case 0:
    uavID = "uav0/";
  case 1:
    uavID = "uav1/";
    break;
  case 2:
    uavID = "uav2/";
    break;
  }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 获取当前时刻
 */
float VIR_SIM_LEADER::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void VIR_SIM_LEADER::fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    fwstates = *msg;
}

void VIR_SIM_LEADER::ros_sub_pub()
{

  vir_leader_pub = nh.advertise<fixed_wing_formation_control::Leaderstates>(
      add2str(uavID, "fixed_wing_formation_control/leader_states"), 10);

  fw_states_sub = nh.subscribe<fixed_wing_formation_control::FWstates>(
      add2str(uavID, "fixed_wing_formation_control/fw_states"), 10,
      &VIR_SIM_LEADER::fw_state_cb, this);
}

void VIR_SIM_LEADER::run(int argc, char **argv)
{
    ros::Rate rate(10.0);
    begin_time = ros::Time::now(); /* 记录启控时间 */
    ros_sub_pub();               //继承而来，只是发布消息

    while (ros::ok())
    {
        current_time = get_ros_time(begin_time);

        VIR_SIM_LEADER_INFO("当前时间：["<<current_time<<"]s");



        leaderstates.airspeed = fwstates.air_speed;       //***这里没有发送角度信息，所以导致编队里边直接使用角度时出问题，不过应该美问题，先用算的试一试
        leaderstates.altitude = fwstates.altitude;
        leaderstates.latitude = fwstates.latitude;
        leaderstates.longitude = fwstates.longitude;
        leaderstates.ned_vel_x = fwstates.ned_vel_x;
        leaderstates.ned_vel_y = fwstates.ned_vel_y;
        leaderstates.ned_vel_z = fwstates.ned_vel_z;
        leaderstates.global_vel_x = fwstates.global_vel_x;
        leaderstates.global_vel_y = fwstates.global_vel_y;
        leaderstates.global_vel_z = fwstates.global_vel_z;
        leaderstates.roll_angle = fwstates.roll_angle;
        leaderstates.pitch_angle = fwstates.pitch_angle;
        leaderstates.yaw_angle = fwstates.yaw_angle;

        //***气压计高度
        leaderstates.relative_alt = fwstates.relative_alt;

        leaderstates.yaw_rate = fwstates.yaw_rate;          //***PS:直接从imu获得，影响这些角速度吗？

       leaderstates.pitch_rate = fwstates.pitch_rate;

       leaderstates.roll_rate = fwstates.roll_rate;

        VIR_SIM_LEADER_INFO("领机信息发送成功");

        vir_leader_pub.publish(leaderstates);

        ros::spinOnce(); //挂起一段时间，保证周期的速度
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vir_sim_leader");

    VIR_SIM_LEADER _vir_leader;

    if (true)
    {
        _vir_leader.run(argc, argv);
    }

    return 0;
}
