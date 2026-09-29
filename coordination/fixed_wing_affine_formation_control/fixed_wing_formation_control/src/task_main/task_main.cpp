/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#include "task_main.hpp"

/**
 * @Input: int id数字
 * @Output: 
 * @Description: 设定当前无人机的ID
 */
void TASK_MAIN::set_planeID(int id) {
  planeID = id;
  
  switch (planeID) {
    /*长机ID*/
  case 0:
    uavID = "uav0/";
    neighborID[0] = "uav1/";
    neighborID[1] = "uav2/";
    break;
  case 1:
    uavID = "uav1/";
    neighborID[0] = "uav0/";
    neighborID[1] = "uav2/";
    break;
  case 2:
    uavID = "uav2/";
    neighborID[0] = "uav0/";
    neighborID[1] = "uav1/";
    break;
    /*僚机ID*/
  case 3:
    uavID = "uav3/";
    break;
  case 4:
    uavID = "uav4/";
    break;
  case 5:
    uavID = "uav5/";
    break;

  }
}


/**
 * @Input: begin_time起始ros时间
 * @Output: float time_now
 * @Description: 获取当前时间，返回当前相对于起始时间的时间差
 */
float TASK_MAIN::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();                       /* ros当前时间赋值 */
    float currTimeSec = time_now.sec - begin.sec;                /* 秒*/
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9; /* 纳秒 */
    return (currTimeSec + currTimenSec);                         /* 返回当前相对于起始时间的时间差 */
}

/* FWstates消息的回调函数，固定翼全部状态量 */
void TASK_MAIN::fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    fwstates = *msg;
}
/* FWstates消息的回调函数，固定翼全部状态量 */
void TASK_MAIN::fw_state_cb0(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    everyfwstates0 = *msg;
}
/* FWstates消息的回调函数，固定翼全部状态量 */
void TASK_MAIN::fw_state_cb1(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    everyfwstates1 = *msg;
}
/* FWstates消息的回调函数，固定翼全部状态量 */
void TASK_MAIN::fw_state_cb2(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    everyfwstates2 = *msg;
}
/* FWstates消息的回调函数，固定翼全部状态量 */
void TASK_MAIN::fw_state_cb3(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    everyfwstates3 = *msg;
}
/* FWstates消息的回调函数，固定翼全部状态量 */
void TASK_MAIN::fw_state_cb4(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    everyfwstates4 = *msg;
}
/* FWstates消息的回调函数，固定翼全部状态量 */
void TASK_MAIN::fw_state_cb5(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    everyfwstates5 = *msg;
}
/* FWstates消息的回调函数，订阅长机的邻居长机信息状态量 */
void TASK_MAIN::neighbor_cb0(const fixed_wing_formation_control::FWcmd::ConstPtr &msg)
{
    neighbor0 = *msg;
}
/* FWstates消息的回调函数，订阅长机的邻居长机信息状态量 */
void TASK_MAIN::neighbor_cb1(const fixed_wing_formation_control::FWcmd::ConstPtr &msg)
{
    neighbor1 = *msg;
}
/* FWstates消息的回调函数，订阅僚机的长机信息状态量 */
void TASK_MAIN::leader_states_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    leaderstates = *msg;
}
/* Fwmonitor消息的回调函数，监控节点飞机以及任务状态 */
void TASK_MAIN::fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg)
{
    fwmonitor_flag = *msg;
}
/* Fw_cmd_mode消息的回调函数，commander指定比赛模式 */
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

/*################################### 订阅subscribe #####################################*/ 

  fw_states_sub = 
        nh.subscribe                                                   /*【订阅】当前固定翼全部状态量,这里订阅了全部的从机 */
        <fixed_wing_formation_control::FWstates>(                      /* ------由pack_fw_states.cpp发布 */
            add2str(uavID, "fixed_wing_formation_control/fw_states"), 10, 
            &TASK_MAIN::fw_state_cb, this);     
    

  every_fw_states_sub0 = 
        nh.subscribe                                                   /*【订阅】所有固定翼无人机全部状态量,这里订阅了全部的从机 */
         <fixed_wing_formation_control::FWstates>(                      /* ------由pack_fw_states.cpp发布 */
            add2str("uav0/", "fixed_wing_formation_control/fw_states"),10,  
            &TASK_MAIN::fw_state_cb0, this);

  every_fw_states_sub1 = 
        nh.subscribe                                                   /*【订阅】所有固定翼无人机全部状态量,这里订阅了全部的从机 */
        <fixed_wing_formation_control::FWstates>(                      /* ------由pack_fw_states.cpp发布 */
            add2str("uav1/", "fixed_wing_formation_control/fw_states"),10,  
            &TASK_MAIN::fw_state_cb1, this);
  every_fw_states_sub2 = 
        nh.subscribe                                                   /*【订阅】所有固定翼无人机全部状态量,这里订阅了全部的从机 */
        <fixed_wing_formation_control::FWstates>(                      /* ------由pack_fw_states.cpp发布 */
            add2str("uav2/", "fixed_wing_formation_control/fw_states"),10,  
            &TASK_MAIN::fw_state_cb2, this);
  every_fw_states_sub3 = 
        nh.subscribe                                                   /*【订阅】所有固定翼无人机全部状态量,这里订阅了全部的从机 */
        <fixed_wing_formation_control::FWstates>(                      /* ------由pack_fw_states.cpp发布 */
            add2str("uav3/", "fixed_wing_formation_control/fw_states"),10,  
            &TASK_MAIN::fw_state_cb3, this);
  every_fw_states_sub4 = 
        nh.subscribe                                                   /*【订阅】所有固定翼无人机全部状态量,这里订阅了全部的从机 */
        <fixed_wing_formation_control::FWstates>(                      /* ------由pack_fw_states.cpp发布 */
            add2str("uav4/", "fixed_wing_formation_control/fw_states"),10,  
            &TASK_MAIN::fw_state_cb4, this);
  every_fw_states_sub5 = 
        nh.subscribe                                                   /*【订阅】所有固定翼无人机全部状态量,这里订阅了全部的从机 */
        <fixed_wing_formation_control::FWstates>(                      /* ------由pack_fw_states.cpp发布 */
            add2str("uav5/", "fixed_wing_formation_control/fw_states"),10,  
            &TASK_MAIN::fw_state_cb5, this);                
                    
  fwmonitor_sub =
        nh.subscribe                                                              /* 【订阅】监控节点飞机以及任务状态 */
        <fixed_wing_formation_control::Fwmonitor>(
            add2str(uavID, "fixed_wing_formation_control/fwmonitor_flags"), 10,
            &TASK_MAIN::fw_fwmonitor_cb, this);

  fw_cmd_mode_sub =
        nh.subscribe                                                              /* 【订阅】commander指定比赛模式 */   
        <fixed_wing_formation_control::Fw_cmd_mode>(                              /* -------由switch_fw_mode.cpp发布*/
            add2str(uavID, "fixed_wing_formation_control/fw_cmd_mode"), 10,             
            &TASK_MAIN::fw_cmd_mode_cb, this);                                          

  neighbor_sub0 =
            nh.subscribe                                                              /* 【订阅】长机邻居的弧长信息 */
            <fixed_wing_formation_control::FWcmd>(                                 /* -------由pack_fw_states.cpp发布 */
                add2str(neighborID[0],"fixed_wing_formation_control/fw_cmd"), 10,
                &TASK_MAIN::neighbor_cb0, this);
  
  neighbor_sub1 =
        nh.subscribe                                                              /* 【订阅】长机邻居的弧长信息 */
        <fixed_wing_formation_control::FWcmd>(                                 /* -------由pack_fw_states.cpp发布 */
            add2str(neighborID[1],"fixed_wing_formation_control/fw_cmd"), 10,
            &TASK_MAIN::neighbor_cb1, this);

  leader_states_sub =
        nh.subscribe                                                              /* 【订阅】对应长机2信息 */
        <fixed_wing_formation_control::FWstates>(                                    /* -------由pack_fw_states.cpp发布，vir_sim_leader.cpp订阅 */
           add2str(leaderID,"fixed_wing_formation_control/fw_states"), 10,
           &TASK_MAIN::leader_states_cb, this);
           

/*################################### 发布advertise #####################################*/ 
  fw_cmd_pub = 
        nh.advertise                                                     /* 【发布】固定翼固定翼控制指令期望值,三个欧拉角和油门 */ 
        <fixed_wing_formation_control::FWcmd>(                           /* -----pack_fw_states.cpp订阅 */
            add2str(uavID, "fixed_wing_formation_control/fw_cmd"), 10);         

  formation_control_states_pub =                                                /* 【发布】编队控制器状态，只起到一个监控作用 */
        nh.advertise                                                           
        <fixed_wing_formation_control::Formation_control_states>(                  
            add2str(uavID,"fixed_wing_formation_control/formation_control_states"),10);

  fw_current_mode_pub =                                                         /* 【发布】飞机当前所处任务阶段,飞机当前控制模式赋值 */
        nh.advertise                                                              
        <fixed_wing_formation_control::Fw_current_mode>(
            add2str(uavID, "fixed_wing_formation_control/fw_current_mode"), 10);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 订阅uav0,uav1邻居或僚机的长机信息
 */


/**
 * @Input: void
 * @Output: void
 * @Description: 编队状态值赋值发布，编队位置误差（机体系和ned），速度误差以及期望空速，gps，期望地速
 */
void TASK_MAIN::formation_states_pub() {
  formation_control_states.planeID = formation_error.planeID;

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
 * @Description: 长机编队控制器主函数，完成对于长机、长机邻居状态的赋值，传入编队控制器
 */
void TASK_MAIN::control_formation()
{
    
    fw_col_mode_current = fwstates.control_mode;
    itsneighbor_scurve0 = neighbor0.scurve; //关键：长机邻居弧长的赋值
    itsneighbor_scurve1 = neighbor1.scurve; //关键：长机邻居弧长的赋值
    TASK_MAIN_INFO("control_formation/当前订阅的无人机uavID = " << uavID << ";" << "订阅的长机邻居neighbor0.planeID = " << neighbor0.planeID << ";"<< "neighbor1.planeID = " << neighbor1.planeID << ";");
    TASK_MAIN_INFO("control_formation/itsneighbor_scurve0 = " << itsneighbor_scurve0 << ";"<<"itsneighbor_scurve1 = "<<itsneighbor_scurve1<<";");

    /* 本机状态赋值 */
    thisfw_states.flight_mode = fwstates.control_mode;
    
    /* 自定义状态赋值 */
    thisfw_states.air_speed = fwstates.air_speed;/* 长机空速赋值 */
    thisfw_states.in_air = fwstates.in_air;/* 长机起飞状态标志位赋值 */
    thisfw_states.scurve = fwstates.scurve;/* 虚拟目标点弧长赋值 */
    thisfw_states.planeID = fwstates.planeID;


    /* 经纬度赋值 */
    thisfw_states.altitude = fwstates.relative_alt;
    thisfw_states.altitude_lock = true; /* 保证TECS */
    thisfw_states.in_air = true;        /* 保证tecs */
    thisfw_states.latitude = fwstates.latitude;
    thisfw_states.longitude = fwstates.longitude;
    TASK_MAIN_INFO("control_formation/fwstates.latitude = " << fwstates.latitude << ";"<<"fwstates.longitude = "<<fwstates.longitude<<";");
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
    TASK_MAIN_INFO("订阅leader航向角"<<thisfw_states.yaw_angle<<";");
    thisfw_states.att_quat[0] = fwstates.att_quater.w;
    thisfw_states.att_quat[1] = fwstates.att_quater.x;
    thisfw_states.att_quat[2] = fwstates.att_quater.y;
    thisfw_states.att_quat[3] = fwstates.att_quater.z;
    quat_2_rotmax(thisfw_states.att_quat, thisfw_states.rotmat);

    thisfw_states.body_acc[0] = fwstates.body_acc_x;
    thisfw_states.body_acc[1] = fwstates.body_acc_y;
    thisfw_states.body_acc[2] = fwstates.body_acc_z;
    matrix_plus_vector_3(thisfw_states.ned_acc, thisfw_states.rotmat, thisfw_states.body_acc);

    thisfw_states.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***长机航向角速度赋值
    thisfw_states.yaw_rate = fwstates.yaw_rate;

    //***设定无人机编号
    formation_controller.set_fw_planeID(fwstates.planeID);
    formation_controller.set_ID(fwstates.planeID);

    /* 设定编队形状 */
    //***可在此处添加编队改变的键盘输入命令
    formation_controller.set_formation_type(formation_type_id);

    /* 模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！ */
    if (fw_col_mode_current != fw_col_mode_last)
    {
       formation_controller.reset_formation_controller();
    }
    /* 更新长机和长机邻居状态 */
    formation_controller.update_leaders_states(current_time,&itsneighbor_scurve0,&itsneighbor_scurve1,&thisfw_states);
    
    /* 编队控制，使用导航律的方法 */

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
    fw_4cmd.scurve = formation_cmd.scurve;
    fw_4cmd.planeID = formation_cmd.planeID;


    TASK_MAIN_INFO("控制量(度): throttle, roll, pitch, yaw, scurve: " << fw_4cmd.throttle_sp << ";" << rad_2_deg(fw_4cmd.roll_angle_sp) << ";" << rad_2_deg(fw_4cmd.pitch_angle_sp)<< ";" <<rad_2_deg(fw_4cmd.yaw_angle_sp)<<";"<<fw_4cmd.scurve);
    TASK_MAIN_INFO("FWcmd中的planeID:"<<fw_4cmd.planeID<<";");


    fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */
    formation_states_pub();      /* 发布编队控制器状态 */

    fw_col_mode_last = fw_col_mode_current; /* 上一次模式的继承 */
  
}

/**
 * @Input: void
 * @Output: void
 * @Description: 编队控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */
void TASK_MAIN::follower_control_formation()
{

    TASK_MAIN_INFO("follower_control_formation/当前订阅的无人机uavID = " << uavID << ";" );
                    /*uav0*/
    everyfw_states0.planeID = everyfwstates0.planeID;
    everyfw_states0.flight_mode = everyfwstates0.control_mode;
    /* 自定义状态赋值 */
    everyfw_states0.air_speed = everyfwstates0.air_speed;/* 长机空速赋值 */
    everyfw_states0.in_air = everyfwstates0.in_air;/* 长机起飞状态标志位赋值 */
    everyfw_states0.scurve = everyfwstates0.scurve;/* 虚拟目标点弧长赋值 */

    /* 经纬度赋值 */
    everyfw_states0.altitude = everyfwstates0.relative_alt;
    everyfw_states0.altitude_lock = true; /* 保证TECS */
    everyfw_states0.in_air = true;        /* 保证tecs */
    everyfw_states0.latitude = everyfwstates0.latitude;
    everyfw_states0.longitude = everyfwstates0.longitude;

    TASK_MAIN_INFO("follower_control_formation/everyfwstates0.latitude = " << everyfwstates0.latitude << ";"<<"everyfwstates0.longitude = "<<everyfwstates0.longitude<<";");

    everyfw_states0.relative_alt = everyfwstates0.relative_alt;

    everyfw_states0.ned_vel_x = everyfwstates0.ned_vel_x;
    everyfw_states0.ned_vel_y = everyfwstates0.ned_vel_y;
    everyfw_states0.ned_vel_z = everyfwstates0.ned_vel_z;

    everyfw_states0.global_vel_x = everyfwstates0.global_vel_x;
    everyfw_states0.global_vel_y = everyfwstates0.global_vel_y;
    everyfw_states0.global_vel_z = everyfwstates0.global_vel_z;

    everyfw_states0.pitch_angle = everyfwstates0.pitch_angle;
    everyfw_states0.roll_angle = everyfwstates0.roll_angle;
    everyfw_states0.yaw_angle = everyfwstates0.yaw_angle;
    
    everyfw_states0.att_quat[0] = everyfwstates0.att_quater.w;
    everyfw_states0.att_quat[1] = everyfwstates0.att_quater.x;
    everyfw_states0.att_quat[2] = everyfwstates0.att_quater.y;
    everyfw_states0.att_quat[3] = everyfwstates0.att_quater.z;
    quat_2_rotmax(everyfw_states0.att_quat, everyfw_states0.rotmat);

    everyfw_states0.body_acc[0] = everyfwstates0.body_acc_x;
    everyfw_states0.body_acc[1] = everyfwstates0.body_acc_y;
    everyfw_states0.body_acc[2] = everyfwstates0.body_acc_z;
    matrix_plus_vector_3(everyfw_states0.ned_acc, everyfw_states0.rotmat, everyfw_states0.body_acc);
    everyfw_states0.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***长机航向角速度赋值
    everyfw_states0.yaw_rate = everyfwstates0.yaw_rate;


                    /*uav1*/
    everyfw_states1.planeID = everyfwstates1.planeID;
    everyfw_states1.flight_mode = everyfwstates1.control_mode;
    /* 自定义状态赋值 */
    everyfw_states1.air_speed = everyfwstates1.air_speed;/* 长机空速赋值 */
    everyfw_states1.in_air = everyfwstates1.in_air;/* 长机起飞状态标志位赋值 */
    everyfw_states1.scurve = everyfwstates1.scurve;/* 虚拟目标点弧长赋值 */

    /* 经纬度赋值 */
    everyfw_states1.altitude = everyfwstates1.relative_alt;
    everyfw_states1.altitude_lock = true; /* 保证TECS */
    everyfw_states1.in_air = true;        /* 保证tecs */
    everyfw_states1.latitude = everyfwstates1.latitude;
    everyfw_states1.longitude = everyfwstates1.longitude;

    TASK_MAIN_INFO("follower_control_formation/everyfwstates1.latitude = " << everyfwstates1.latitude << ";"<<"everyfwstates1.longitude = "<<everyfwstates1.longitude<<";");

    everyfw_states1.relative_alt = everyfwstates1.relative_alt;

    everyfw_states1.ned_vel_x = everyfwstates1.ned_vel_x;
    everyfw_states1.ned_vel_y = everyfwstates1.ned_vel_y;
    everyfw_states1.ned_vel_z = everyfwstates1.ned_vel_z;

    everyfw_states1.global_vel_x = everyfwstates1.global_vel_x;
    everyfw_states1.global_vel_y = everyfwstates1.global_vel_y;
    everyfw_states1.global_vel_z = everyfwstates1.global_vel_z;

    everyfw_states1.pitch_angle = everyfwstates1.pitch_angle;
    everyfw_states1.roll_angle = everyfwstates1.roll_angle;
    everyfw_states1.yaw_angle = everyfwstates1.yaw_angle;
    
    everyfw_states1.att_quat[0] = everyfwstates1.att_quater.w;
    everyfw_states1.att_quat[1] = everyfwstates1.att_quater.x;
    everyfw_states1.att_quat[2] = everyfwstates1.att_quater.y;
    everyfw_states1.att_quat[3] = everyfwstates1.att_quater.z;
    quat_2_rotmax(everyfw_states1.att_quat, everyfw_states1.rotmat);

    everyfw_states1.body_acc[0] = everyfwstates1.body_acc_x;
    everyfw_states1.body_acc[1] = everyfwstates1.body_acc_y;
    everyfw_states1.body_acc[2] = everyfwstates1.body_acc_z;
    matrix_plus_vector_3(everyfw_states1.ned_acc, everyfw_states1.rotmat, everyfw_states1.body_acc);
    everyfw_states1.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***长机航向角速度赋值
    everyfw_states1.yaw_rate = everyfwstates1.yaw_rate;

                    /*uav2*/
    everyfw_states2.planeID = everyfwstates2.planeID;
    everyfw_states2.flight_mode = everyfwstates2.control_mode;
    /* 自定义状态赋值 */
    everyfw_states2.air_speed = everyfwstates2.air_speed;/* 长机空速赋值 */
    everyfw_states2.in_air = everyfwstates2.in_air;/* 长机起飞状态标志位赋值 */
    everyfw_states2.scurve = everyfwstates2.scurve;/* 虚拟目标点弧长赋值 */

    /* 经纬度赋值 */
    everyfw_states2.altitude = everyfwstates2.relative_alt;
    everyfw_states2.altitude_lock = true; /* 保证TECS */
    everyfw_states2.in_air = true;        /* 保证tecs */
    everyfw_states2.latitude = everyfwstates2.latitude;
    everyfw_states2.longitude = everyfwstates2.longitude;

    TASK_MAIN_INFO("follower_control_formation/everyfwstates2.latitude = " << everyfwstates2.latitude << ";"<<"everyfwstates2.longitude = "<<everyfwstates2.longitude<<";");

    everyfw_states2.relative_alt = everyfwstates2.relative_alt;

    everyfw_states2.ned_vel_x = everyfwstates2.ned_vel_x;
    everyfw_states2.ned_vel_y = everyfwstates2.ned_vel_y;
    everyfw_states2.ned_vel_z = everyfwstates2.ned_vel_z;

    everyfw_states2.global_vel_x = everyfwstates2.global_vel_x;
    everyfw_states2.global_vel_y = everyfwstates2.global_vel_y;
    everyfw_states2.global_vel_z = everyfwstates2.global_vel_z;

    everyfw_states2.pitch_angle = everyfwstates2.pitch_angle;
    everyfw_states2.roll_angle = everyfwstates2.roll_angle;
    everyfw_states2.yaw_angle = everyfwstates2.yaw_angle;
    
    everyfw_states2.att_quat[0] = everyfwstates2.att_quater.w;
    everyfw_states2.att_quat[1] = everyfwstates2.att_quater.x;
    everyfw_states2.att_quat[2] = everyfwstates2.att_quater.y;
    everyfw_states2.att_quat[3] = everyfwstates2.att_quater.z;
    quat_2_rotmax(everyfw_states2.att_quat, everyfw_states2.rotmat);

    everyfw_states2.body_acc[0] = everyfwstates2.body_acc_x;
    everyfw_states2.body_acc[1] = everyfwstates2.body_acc_y;
    everyfw_states2.body_acc[2] = everyfwstates2.body_acc_z;
    matrix_plus_vector_3(everyfw_states2.ned_acc, everyfw_states2.rotmat, everyfw_states2.body_acc);
    everyfw_states2.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***长机航向角速度赋值
    everyfw_states2.yaw_rate = everyfwstates2.yaw_rate;

                        /*uav3*/
    everyfw_states3.planeID = everyfwstates3.planeID;
    everyfw_states3.flight_mode = everyfwstates3.control_mode;
    /* 自定义状态赋值 */
    everyfw_states3.air_speed = everyfwstates3.air_speed;/* 长机空速赋值 */
    everyfw_states3.in_air = everyfwstates3.in_air;/* 长机起飞状态标志位赋值 */
    everyfw_states3.scurve = everyfwstates3.scurve;/* 虚拟目标点弧长赋值 */

    /* 经纬度赋值 */
    everyfw_states3.altitude = everyfwstates3.relative_alt;
    everyfw_states3.altitude_lock = true; /* 保证TECS */
    everyfw_states3.in_air = true;        /* 保证tecs */
    everyfw_states3.latitude = everyfwstates3.latitude;
    everyfw_states3.longitude = everyfwstates3.longitude;

    TASK_MAIN_INFO("follower_control_formation/everyfwstates3.latitude = " << everyfwstates3.latitude << ";"<<"everyfwstates3.longitude = "<<everyfwstates3.longitude<<";");

    everyfw_states3.relative_alt = everyfwstates3.relative_alt;

    everyfw_states3.ned_vel_x = everyfwstates3.ned_vel_x;
    everyfw_states3.ned_vel_y = everyfwstates3.ned_vel_y;
    everyfw_states3.ned_vel_z = everyfwstates3.ned_vel_z;

    everyfw_states3.global_vel_x = everyfwstates3.global_vel_x;
    everyfw_states3.global_vel_y = everyfwstates3.global_vel_y;
    everyfw_states3.global_vel_z = everyfwstates3.global_vel_z;

    everyfw_states3.pitch_angle = everyfwstates3.pitch_angle;
    everyfw_states3.roll_angle = everyfwstates3.roll_angle;
    everyfw_states3.yaw_angle = everyfwstates3.yaw_angle;
    
    everyfw_states3.att_quat[0] = everyfwstates3.att_quater.w;
    everyfw_states3.att_quat[1] = everyfwstates3.att_quater.x;
    everyfw_states3.att_quat[2] = everyfwstates3.att_quater.y;
    everyfw_states3.att_quat[3] = everyfwstates3.att_quater.z;
    quat_2_rotmax(everyfw_states3.att_quat, everyfw_states3.rotmat);

    everyfw_states3.body_acc[0] = everyfwstates3.body_acc_x;
    everyfw_states3.body_acc[1] = everyfwstates3.body_acc_y;
    everyfw_states3.body_acc[2] = everyfwstates3.body_acc_z;
    matrix_plus_vector_3(everyfw_states3.ned_acc, everyfw_states3.rotmat, everyfw_states3.body_acc);
    everyfw_states3.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***长机航向角速度赋值
    everyfw_states3.yaw_rate = everyfwstates3.yaw_rate;

                    /*uav4*/
    everyfw_states4.planeID = everyfwstates4.planeID;
    everyfw_states4.flight_mode = everyfwstates4.control_mode;
    /* 自定义状态赋值 */
    everyfw_states4.air_speed = everyfwstates4.air_speed;/* 长机空速赋值 */
    everyfw_states4.in_air = everyfwstates4.in_air;/* 长机起飞状态标志位赋值 */
    everyfw_states4.scurve = everyfwstates4.scurve;/* 虚拟目标点弧长赋值 */

    /* 经纬度赋值 */
    everyfw_states4.altitude = everyfwstates4.relative_alt;
    everyfw_states4.altitude_lock = true; /* 保证TECS */
    everyfw_states4.in_air = true;        /* 保证tecs */
    everyfw_states4.latitude = everyfwstates4.latitude;
    everyfw_states4.longitude = everyfwstates4.longitude;

    TASK_MAIN_INFO("follower_control_formation/everyfwstates4.latitude = " << everyfwstates4.latitude << ";"<<"everyfwstates4.longitude = "<<everyfwstates4.longitude<<";");

    everyfw_states4.relative_alt = everyfwstates4.relative_alt;

    everyfw_states4.ned_vel_x = everyfwstates4.ned_vel_x;
    everyfw_states4.ned_vel_y = everyfwstates4.ned_vel_y;
    everyfw_states4.ned_vel_z = everyfwstates4.ned_vel_z;

    everyfw_states4.global_vel_x = everyfwstates4.global_vel_x;
    everyfw_states4.global_vel_y = everyfwstates4.global_vel_y;
    everyfw_states4.global_vel_z = everyfwstates4.global_vel_z;

    everyfw_states4.pitch_angle = everyfwstates4.pitch_angle;
    everyfw_states4.roll_angle = everyfwstates4.roll_angle;
    everyfw_states4.yaw_angle = everyfwstates4.yaw_angle;
    
    everyfw_states4.att_quat[0] = everyfwstates4.att_quater.w;
    everyfw_states4.att_quat[1] = everyfwstates4.att_quater.x;
    everyfw_states4.att_quat[2] = everyfwstates4.att_quater.y;
    everyfw_states4.att_quat[3] = everyfwstates4.att_quater.z;
    quat_2_rotmax(everyfw_states4.att_quat, everyfw_states4.rotmat);

    everyfw_states4.body_acc[0] = everyfwstates4.body_acc_x;
    everyfw_states4.body_acc[1] = everyfwstates4.body_acc_y;
    everyfw_states4.body_acc[2] = everyfwstates4.body_acc_z;
    matrix_plus_vector_3(everyfw_states4.ned_acc, everyfw_states4.rotmat, everyfw_states4.body_acc);
    everyfw_states4.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***长机航向角速度赋值
    everyfw_states4.yaw_rate = everyfwstates4.yaw_rate;

                    /*uav5*/
    everyfw_states5.planeID = everyfwstates5.planeID;
    everyfw_states5.flight_mode = everyfwstates5.control_mode;
    /* 自定义状态赋值 */
    everyfw_states5.air_speed = everyfwstates5.air_speed;/* 长机空速赋值 */
    everyfw_states5.in_air = everyfwstates5.in_air;/* 长机起飞状态标志位赋值 */
    everyfw_states5.scurve = everyfwstates5.scurve;/* 虚拟目标点弧长赋值 */

    /* 经纬度赋值 */
    everyfw_states5.altitude = everyfwstates5.relative_alt;
    everyfw_states5.altitude_lock = true; /* 保证TECS */
    everyfw_states5.in_air = true;        /* 保证tecs */
    everyfw_states5.latitude = everyfwstates5.latitude;
    everyfw_states5.longitude = everyfwstates5.longitude;

    TASK_MAIN_INFO("follower_control_formation/everyfwstates5.latitude = " << everyfwstates5.latitude << ";"<<"everyfwstates5.longitude = "<<everyfwstates5.longitude<<";");

    everyfw_states5.relative_alt = everyfwstates5.relative_alt;

    everyfw_states5.ned_vel_x = everyfwstates5.ned_vel_x;
    everyfw_states5.ned_vel_y = everyfwstates5.ned_vel_y;
    everyfw_states5.ned_vel_z = everyfwstates5.ned_vel_z;

    everyfw_states5.global_vel_x = everyfwstates5.global_vel_x;
    everyfw_states5.global_vel_y = everyfwstates5.global_vel_y;
    everyfw_states5.global_vel_z = everyfwstates5.global_vel_z;

    everyfw_states5.pitch_angle = everyfwstates5.pitch_angle;
    everyfw_states5.roll_angle = everyfwstates5.roll_angle;
    everyfw_states5.yaw_angle = everyfwstates5.yaw_angle;
    
    everyfw_states5.att_quat[0] = everyfwstates5.att_quater.w;
    everyfw_states5.att_quat[1] = everyfwstates5.att_quater.x;
    everyfw_states5.att_quat[2] = everyfwstates5.att_quater.y;
    everyfw_states5.att_quat[3] = everyfwstates5.att_quater.z;
    quat_2_rotmax(everyfw_states5.att_quat, everyfw_states5.rotmat);

    everyfw_states5.body_acc[0] = everyfwstates5.body_acc_x;
    everyfw_states5.body_acc[1] = everyfwstates5.body_acc_y;
    everyfw_states5.body_acc[2] = everyfwstates5.body_acc_z;
    matrix_plus_vector_3(everyfw_states5.ned_acc, everyfw_states5.rotmat, everyfw_states5.body_acc);
    everyfw_states5.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
   
    //***长机航向角速度赋值
    everyfw_states5.yaw_rate = everyfwstates5.yaw_rate;

    TASK_MAIN_INFO("leader航向角"<<everyfw_states0.yaw_angle<<";"<<everyfw_states1.yaw_angle<<";"<<everyfw_states2.yaw_angle<<";");


                        /* 本机状态赋值 */
    fw_col_mode_current = fwstates.control_mode;
    thisfw_states.flight_mode = fwstates.control_mode;
    /* 自定义状态赋值 */
    thisfw_states.air_speed = fwstates.air_speed;/* 长机空速赋值 */
    thisfw_states.in_air = fwstates.in_air;/* 长机起飞状态标志位赋值 */
    thisfw_states.scurve = fwstates.scurve;/* 虚拟目标点弧长赋值 */
    thisfw_states.planeID = fwstates.planeID;

    /* 经纬度赋值 */
    thisfw_states.altitude = fwstates.relative_alt;
    thisfw_states.altitude_lock = true; /* 保证TECS */
    thisfw_states.in_air = true;        /* 保证tecs */
    thisfw_states.latitude = fwstates.latitude;
    thisfw_states.longitude = fwstates.longitude;
    TASK_MAIN_INFO( "僚机ID = " << fwstates.planeID << ";");
    TASK_MAIN_INFO("follower_control_formation/fwstates.latitude = " << fwstates.latitude << ";"<<"fwstates.longitude = "<<fwstates.longitude<<";");

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

    thisfw_states.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */
    
    //***僚机航向角速度赋值
    thisfw_states.yaw_rate = fwstates.yaw_rate;

    //***设定僚机编号
    formation_controller.set_fw_planeID(planeID);
    formation_controller.set_ID(fwstates.planeID);
    /* 设定编队形状 */
    //***可在此处添加编队改变的键盘输入命令，本仿真不涉及队形变化
    formation_controller.set_formation_type(formation_type_id);

    /* 模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！ */
    if (fw_col_mode_current != fw_col_mode_last)
    {
       formation_controller.reset_formation_controller();
    }
    /* 更新所有无人机状态 */
    formation_controller.update_everyfw_states(&everyfw_states0, &everyfw_states1, &everyfw_states2, &everyfw_states3, &everyfw_states4, &everyfw_states5, &thisfw_states);

    /* 编队控制，使用导航律的方法 */
    formation_controller.follower_control_law();
    

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
    fw_4cmd.scurve = formation_cmd.scurve;

    TASK_MAIN_INFO("控制量(度): throttle, roll, pitch, yaw, scurve: " << fw_4cmd.throttle_sp << ";" << rad_2_deg(fw_4cmd.roll_angle_sp) << ";" << rad_2_deg(fw_4cmd.pitch_angle_sp)<< ";" <<rad_2_deg(fw_4cmd.yaw_angle_sp)<<";"<<fw_4cmd.scurve);



    fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */
    formation_states_pub();      /* 发布编队控制器状态 */

    fw_col_mode_last = fw_col_mode_current; /* 上一次模式的继承 */
  
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

    //***ros消息订阅与发布调用
    ros_sub_pub();

    while (ros::ok())
    {
        /**
        * 任务大循环，根据来自commander的控制指令来进行响应的控制动作
       */
        TASK_MAIN_INFO("实际上,当前无人机编号planeID = ： " << planeID <<";"<< "uavID = " << uavID << ";");
        if (planeID==0||planeID==1||planeID==2){
            TASK_MAIN_INFO("实际上,当前neighborID1 = ： " << neighborID[0] <<";"<<"neighborID2 = ： " << neighborID[1] << ";");/*长机需要输出邻居无人机信息*/
        }

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
            
            switch (planeID)         /* 执行编队 */
            {
                case 0:
                    
                    formation_type_id = fw_cmd_mode.formation_type;  //***设置编队队形
                    control_formation();
                    fw_4cmd.cmd_mode = "OFFBOARD";
                    fw_cmd_pub.publish(fw_4cmd);
                    TASK_MAIN_INFO("长机命令:");
                    break;
                case 1:
                    formation_type_id = fw_cmd_mode.formation_type;  //***设置编队队形
                    control_formation();
                    fw_4cmd.cmd_mode = "OFFBOARD";
                    fw_cmd_pub.publish(fw_4cmd);
                    TASK_MAIN_INFO("长机命令:");
                    break;
                case 2:
                    formation_type_id = fw_cmd_mode.formation_type;  //***设置编队队形
                    control_formation();
                    fw_4cmd.cmd_mode = "OFFBOARD";
                    fw_cmd_pub.publish(fw_4cmd);
                    TASK_MAIN_INFO("长机命令:");  
                    break;
                case 3:
                    formation_type_id = fw_cmd_mode.formation_type;  //***设置编队队形
                    follower_control_formation();
                    fw_4cmd.cmd_mode = "OFFBOARD";
                    fw_cmd_pub.publish(fw_4cmd);
                    TASK_MAIN_INFO("僚机命令:");
                    break;
                case 4:
                    formation_type_id = fw_cmd_mode.formation_type;  //***设置编队队形
                    follower_control_formation();
                    fw_4cmd.cmd_mode = "OFFBOARD";
                    fw_cmd_pub.publish(fw_4cmd);
                    TASK_MAIN_INFO("僚机命令:");
                    break;
                case 5:
                    formation_type_id = fw_cmd_mode.formation_type;  //***设置编队队形
                    follower_control_formation();
                    fw_4cmd.cmd_mode = "OFFBOARD";
                    fw_cmd_pub.publish(fw_4cmd);
                    TASK_MAIN_INFO("僚机命令:");
                    break;        
                default:
                    break;
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

            TASK_MAIN_INFO("领机不在OFFBOARD模式");
            
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


/* 主函数，调用run */
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
