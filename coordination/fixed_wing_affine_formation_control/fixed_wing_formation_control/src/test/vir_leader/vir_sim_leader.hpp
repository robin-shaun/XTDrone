/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#ifndef VIR_SIM_LEADER_HPP
#define VIR_SIM_LEADER_HPP

#include <ros/ros.h>
#include <fixed_wing_formation_control/FWstates.h>
#include <fixed_wing_formation_control/Leaderstates.h>
#include "../../fixed_wing_lib/syslib.hpp"

#define VIR_SIM_LEADER_INFO(a) cout<<"[]:"<<a<<endl;

class VIR_SIM_LEADER
{
public:
    void run(int argc, char **argv);
    void set_planeID(int id);/* 注意，此处是领机的id，一般设置为0 */

private:
    int planeID{0};
    string uavID{"uav0/"};

    ros::NodeHandle nh;
    ros::Publisher vir_leader_pub;
    ros::Subscriber fw_states_sub;                           /*// 【订阅】固定翼全部状态量*/
    fixed_wing_formation_control::Leaderstates leaderstates; /*//即将要发布的领机的状态*/
    fixed_wing_formation_control::FWstates fwstates;         /*//自定义--飞机打包的全部状态*/

    void ros_sub_pub();
    float get_ros_time(ros::Time begin);
    void show_vir_leader_status();
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg);

    ros::Time begin_time;
    float current_time;
    float last_time;
};

#endif
