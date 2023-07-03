/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#include "task_main.hpp"

/**
 * @Input: int
 * @Output: 
 * @Description: 设定当前飞机的ID
 */
void TASK_MAIN::set_planeID(int id) {
  planeID = id;
  switch (planeID) {
  case 0:
    uavID = "uav0/";
    break;
  case 1:
    uavID = "uav1/";
    break;
  case 2:
    uavID = "uav2/";
    break;
  }
}
/**
 * @Input: ros::Time begin
 * @Output: float time_now
 * @Description: 获取当前时间
 */
float TASK_MAIN::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void TASK_MAIN::fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    fwstates = *msg;
}
void TASK_MAIN::leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg)
{
    leaderstates = *msg;
}
void TASK_MAIN::fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg)
{
    fwmonitor_flag = *msg;
}

void TASK_MAIN::fw_cmd_mode_cb(const fixed_wing_formation_control::Fw_cmd_mode::ConstPtr &msg)
{
    fw_cmd_mode = *msg;
}

/**
 * @Input: void
 * @Output: void
 * @Description: ros的订阅发布声明函数
 */
void TASK_MAIN::ros_sub_pub() {

  fw_states_sub = nh.subscribe /* 【订阅】固定翼全部状态量 */
                  <fixed_wing_formation_control::FWstates>(
                      add2str(uavID, "fixed_wing_formation_control/fw_states"),  //***这里订阅了全部的从机
                      10, &TASK_MAIN::fw_state_cb, this);

  leader_states_sub =
      nh.subscribe /* 【订阅】领机信息 */
      <fixed_wing_formation_control::Leaderstates>(
          add2str(leaderID, "fixed_wing_formation_control/leader_states"), 10,
          &TASK_MAIN::leader_states_cb, this);

  fwmonitor_sub =
      nh.subscribe /* 【订阅】监控节点飞机以及任务状态 */
      <fixed_wing_formation_control::Fwmonitor>(
          add2str(uavID, "fixed_wing_formation_control/fwmonitor_flags"), 10,
          &TASK_MAIN::fw_fwmonitor_cb, this);

  fw_cmd_mode_sub =
      nh.subscribe /* 【订阅】commander指定比赛模式 */   
      <fixed_wing_formation_control::Fw_cmd_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_cmd_mode"), 10,             
          &TASK_MAIN::fw_cmd_mode_cb, this);                          //***switch_fw_mode发布

  fw_cmd_pub = nh.advertise /* 【发布】固定翼四通道控制量 */         //***三个欧拉角和油门
               <fixed_wing_formation_control::FWcmd>(
                   add2str(uavID, "fixed_wing_formation_control/fw_cmd"), 10);         //***pack_fw_state订阅

  formation_control_states_pub =
      nh.advertise /* 【发布】编队控制器状态 */
      <fixed_wing_formation_control::Formation_control_states>(                          //只起到一个监控作用
          add2str(uavID,
                  "fixed_wing_formation_control/formation_control_states"),
          10);

  fw_current_mode_pub =
      nh.advertise /* 【发布】比赛任务进所处阶段 */
      <fixed_wing_formation_control::Fw_current_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_current_mode"), 10);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 编队状态值发布，编队位置误差（机体系和ned），速度误差以及期望空速，gps，期望地速
 */
void TASK_MAIN::formation_states_pub() {
  formation_control_states.planeID = planeID;

  /* 本部分是关于编队的从机的自己与期望值的误差以及领从机偏差的赋值 */
  formation_control_states.err_P_N = formation_error.P_N;
  formation_control_states.err_P_E = formation_error.P_E;
  formation_control_states.err_P_D = formation_error.P_D;
  formation_control_states.err_P_NE = formation_error.P_NE;

  formation_control_states.err_PXb = formation_error.PXb;   //***D系误差  这里借用了后边的一些消息内容,有些地方和消息类型中的定义意义并不一样
  formation_control_states.err_PYb = formation_error.PYb;  //***D系误差
  formation_control_states.err_PZb = formation_error.PZb;  //***D系误差
  formation_control_states.err_VZb = formation_error.Vb;   //***水平速度误差
  formation_control_states.err_VZk = formation_error.Vk;  //***角速度误差
  formation_control_states.err_VXb = formation_error.VXb;  //***水平x速度误差
  formation_control_states.err_VYb = formation_error.VYb;  //***水平y速度误差
  formation_control_states.err_PXk = sqrt(formation_error.PXb * formation_error.PXb + formation_error.PYb * formation_error.PYb);  //***水平位置误差

//   formation_control_states.err_VXb = formation_error.VXb;
//   formation_control_states.err_VYb = formation_error.VYb;
//   formation_control_states.err_VZb = formation_error.VZb;
  formation_control_states.led_fol_vxb = formation_error.led_fol_vxb;
  formation_control_states.led_fol_vyb = formation_error.led_fol_vyb;
  formation_control_states.led_fol_vzb = formation_error.led_fol_vzb;

//   formation_control_states.err_PXk = formation_error.PXk;
  formation_control_states.err_PYk = formation_error.PYk;
  formation_control_states.err_PZk = formation_error.PZk;
  formation_control_states.err_VXk = formation_error.VXk;
  formation_control_states.err_VYk = formation_error.VYk;
//   formation_control_states.err_VZk = formation_error.VZk;
  formation_control_states.led_fol_vxk = formation_error.led_fol_vxk;
  formation_control_states.led_fol_vyk = formation_error.led_fol_vyk;
  formation_control_states.led_fol_vzk = formation_error.led_fol_vzk;

  formation_control_states.led_fol_eta = formation_error.led_fol_eta;   //***D系角度误差
  formation_control_states.eta_deg = rad_2_deg(formation_error.led_fol_eta);

  /* 本部分关于从机的期望值的赋值 */
  formation_control_states.sp_air_speed = formation_sp.air_speed;
  formation_control_states.sp_altitude = formation_sp.altitude;
  formation_control_states.sp_ground_speed = formation_sp.ground_speed;
  formation_control_states.sp_latitude = formation_sp.latitude;
  formation_control_states.sp_longitude = formation_sp.longitude;
  formation_control_states.sp_ned_vel_x = formation_sp.ned_vel_x;
  formation_control_states.sp_ned_vel_y = formation_sp.ned_vel_y;
  formation_control_states.sp_ned_vel_z = formation_sp.ned_vel_z;
  formation_control_states.sp_relative_alt = formation_sp.relative_alt;

  /* 发布编队控制器控制状态 */
  formation_control_states_pub.publish(formation_control_states);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 编队控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */
void TASK_MAIN::control_formation()
{
    fw_col_mode_current = fwstates.control_mode;
    /* 领机状态赋值 */
    leader_states.air_speed = leaderstates.airspeed;

    // leader_states.altitude = leaderstates.altitude;
    //***高度消息看一下
    leader_states.altitude = leaderstates.relative_alt;
    leader_states.latitude = leaderstates.latitude;
    leader_states.longitude = leaderstates.longitude;
    leader_states.relative_alt = leaderstates.relative_alt;

    leader_states.global_vel_x = leaderstates.global_vel_x;
    leader_states.global_vel_y = leaderstates.global_vel_y;
    leader_states.global_vel_z = leaderstates.global_vel_z;

    leader_states.ned_vel_x = leaderstates.ned_vel_x;
    leader_states.ned_vel_y = leaderstates.ned_vel_y;
    leader_states.ned_vel_z = leaderstates.ned_vel_z;

    leader_states.pitch_angle = leaderstates.pitch_angle;
    leader_states.roll_angle = leaderstates.roll_angle;
    leader_states.yaw_angle = leaderstates.yaw_angle;
    leader_states.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***航向角速度赋值
    leader_states.yaw_rate = leaderstates.yaw_rate;

    /* 从机状态赋值 */
    thisfw_states.flight_mode = fwstates.control_mode;

    thisfw_states.air_speed = fwstates.air_speed;
    thisfw_states.in_air = fwstates.in_air;

    // thisfw_states.altitude = fwstates.altitude;
    //***试一下这些高度都用气压计的，也就是
    thisfw_states.altitude = fwstates.relative_alt;
    thisfw_states.altitude_lock = true; /* 保证TECS */
    thisfw_states.in_air = true;        /* 保证tecs */
    thisfw_states.latitude = fwstates.latitude;
    thisfw_states.longitude = fwstates.longitude;

    thisfw_states.relative_alt = fwstates.relative_alt;

    thisfw_states.ned_vel_x = fwstates.ned_vel_x;
    thisfw_states.ned_vel_y = fwstates.ned_vel_y;
    thisfw_states.ned_vel_z = fwstates.ned_vel_z;

    thisfw_states.global_vel_x = fwstates.global_vel_x;
    thisfw_states.global_vel_y = fwstates.global_vel_y;
    thisfw_states.global_vel_z = fwstates.global_vel_z;

    thisfw_states.pitch_angle = fwstates.pitch_angle;
    thisfw_states.roll_angle = fwstates.roll_angle;
    thisfw_states.yaw_angle = fwstates.yaw_angle;
    thisfw_states.att_quat[0] = fwstates.att_quater.w;
    thisfw_states.att_quat[1] = fwstates.att_quater.x;
    thisfw_states.att_quat[2] = fwstates.att_quater.y;
    thisfw_states.att_quat[3] = fwstates.att_quater.z;
    quat_2_rotmax(thisfw_states.att_quat, thisfw_states.rotmat);

    thisfw_states.body_acc[0] = fwstates.body_acc_x;
    thisfw_states.body_acc[1] = fwstates.body_acc_y;
    thisfw_states.body_acc[2] = fwstates.body_acc_z;
    matrix_plus_vector_3(thisfw_states.ned_acc, thisfw_states.rotmat, thisfw_states.body_acc);

    thisfw_states.wind_estimate_x = fwstates.wind_estimate_x;
    thisfw_states.wind_estimate_y = fwstates.wind_estimate_y;
    thisfw_states.wind_estimate_z = fwstates.wind_estimate_z;

    //***航向角速度
    thisfw_states.yaw_rate = fwstates.yaw_rate;

    //***设定从机编号
    formation_controller.set_fw_planeID(planeID);

    /* 设定编队形状 */
    //***可在此处添加编队改变的键盘输入命令
    formation_controller.set_formation_type(formation_type_id);

    /* 模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！ */
    if (fw_col_mode_current != fw_col_mode_last)
    {
       formation_controller.reset_formation_controller();
    }
    /* 更新飞机状态，领机状态 */
    formation_controller.update_led_fol_states(&leader_states, &thisfw_states);
    /* 编队控制 */
    //*** 使用导航律的方法
    //***这块也有消息提示，可以if设置完队形之后再说？或者将队形设置放在模式切换里，然后当消息传递进来，应该都行
    formation_controller.control_law();

    /* 获得最终控制量 */
    formation_controller.get_formation_4cmd(formation_cmd);
    /* 获得编队控制期望值 */
    formation_controller.get_formation_sp(formation_sp);
    /* 获得编队误差信息 */
    formation_controller.get_formation_error(formation_error);

    /* 控制量赋值 */
    fw_4cmd.throttle_sp = formation_cmd.thrust;
    fw_4cmd.roll_angle_sp = formation_cmd.roll;
    fw_4cmd.pitch_angle_sp = formation_cmd.pitch;
    fw_4cmd.yaw_angle_sp = formation_cmd.yaw;

    TASK_MAIN_INFO("控制量(度): throttle, roll, pitch, yaw: " << fw_4cmd.throttle_sp << ";" << rad_2_deg(fw_4cmd.roll_angle_sp) << ";" << rad_2_deg(fw_4cmd.pitch_angle_sp)<< ";" <<rad_2_deg(fw_4cmd.yaw_angle_sp));



    fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */
    formation_states_pub();      /* 发布编队控制器状态 */

    fw_col_mode_last = fw_col_mode_current; /* 上一次模式的纪录 */
}


/**
 * @Input: void
 * @Output: void
 * @Description: 主循环
 */
void TASK_MAIN::run()
{
    ros::Rate rate(50.0);
    begin_time = ros::Time::now(); /* 记录启控时间 */

    //***ros调用
    ros_sub_pub();

    while (ros::ok())
    {
        /**
        * 任务大循环，根据来自commander的控制指令来进行响应的控制动作
       */
        current_time = get_ros_time(begin_time); /*此时刻，只作为纪录，不用于控制*/
        TASK_MAIN_INFO("Time:" << current_time);

        if (!fw_cmd_mode.need_take_off &&
            !fw_cmd_mode.need_formation &&
            !fw_cmd_mode.need_land &&
            !fw_cmd_mode.need_idel &&
            fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("保护子程序");
            /**
             * TODO:保护子程序
             */
            fw_current_mode.mode =
                fixed_wing_formation_control::Fw_current_mode::FW_IN_PROTECT;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("空闲子程序");
            /**
                 * TODO:空闲子程序
                */
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_IDEL;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("降落子程序");
            /**
                 * TODO:降落子程序
                */
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_LANDING;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("编队子程序");
            /**
                 * TODO:虽然完成了节点参数的输入函数以及各个通路，但是节点的参数并没有加载进来
                */
            if(planeID != 0)         //***只有从机体执行编队
            {
                formation_type_id = fw_cmd_mode.formation_type;  //***设置编队队形
                TASK_MAIN_INFO("当前队形：" << formation_type_id);
                control_formation();
            }
            else{               //***否则只改变领机的模式命令
                    fw_4cmd.cmd_mode = "OFFBOARD";
                    fw_cmd_pub.publish(fw_4cmd);
                    TASK_MAIN_INFO("领机命令");
            }

            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_FORMATION;
        }
        else if (fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected)
        {
            TASK_MAIN_INFO("起飞子程序");
            /**
                 * TODO:起飞子程序
                */
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_TAKEOFF;
        }
        else
        {
            if(planeID !=0)
            {
               TASK_MAIN_INFO("错误，飞机当前状态有误");
            }
            else
            {
                TASK_MAIN_INFO("领机不在OFFBOARD模式");
            }
        }

        /**
         * 发布飞机当前状态
        */
        fw_current_mode_pub.publish(fw_current_mode);

        ros::spinOnce();
        rate.sleep();
    }

    return;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "task_main");
    TASK_MAIN _task_main;
    if (true)
    {
        _task_main.run();
    }
    return 0;
}
