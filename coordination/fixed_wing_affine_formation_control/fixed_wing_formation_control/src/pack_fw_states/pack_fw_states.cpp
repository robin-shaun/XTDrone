/*
 * @------------------------------------------1: 1------------------------------------------@
* @修改自  lee-shun Email: 2015097272@qq.com
 * @Description:
 *  * 本程序的作用是：
 *  1. 将来自于mavros的消息坐标变换后打包成Fw_state消息，便于以后使用。
 *  2. 将需要发送给飞机的四通道控制量消息坐标变换解包，发给mavros   
 * //***注意：目前针对复合翼的模式切换，除了offboard外，都是在QGC内完成的(以mission模式起飞，然后键盘切换到offboard)，后续可以进一步完善XTDrone的键盘控制部分


 */

#include "pack_fw_states.hpp"

void PACK_FW_STATES::set_planeID(int id) {
  planeID = id;
  /*变量在pack_fw_states中传出*/
  switch (id) {
  case 0:
    uavID = "uav0/"; /* 领机 */
    break;
  case 1:
    uavID = "uav1/"; /* 领机 */
    break;
  case 2:
    uavID = "uav2/"; /* 领机 */
    break;
  case 3:
    uavID = "uav3/"; /* 从机 */
    break;
  case 4:
    uavID = "uav4/"; /* 从机 */
    break;
  case 5:
    uavID = "uav5/"; /* 从机 */
    break;
  }
}


void PACK_FW_STATES::set_scurve(float scurve) {
    
  fixed_wing_sub_pub.fw_states_form_mavros.scurve = scurve;

}

void PACK_FW_STATES::ros_sub_and_pub() {

  //############################### 订阅mavros消息+控制器指令cmd #################################//
  fixed_wing_states_sub                         /* 【订阅】当前无人机状态 */
      = nh.subscribe<mavros_msgs::State>        /* arm预备，manual手动等*/
        (add2str(uavID, "mavros/state"), 10, &_FIXED_WING_SUB_PUB::state_cb,
         &fixed_wing_sub_pub);

  fixed_wing_imu_sub                            /* 【订阅】无人机imu数据，惯性传感器，飞控计算的方向 */
      = nh.subscribe<sensor_msgs::Imu>          /* 从imu获得姿态四元数、角速度(角速度的正负看看怎么定义?)、线速度*/
        (add2str(uavID, "mavros/imu/data"), 10, &_FIXED_WING_SUB_PUB::imu_cb,
         &fixed_wing_sub_pub);

  fixed_wing_global_position_form_px4_sub       /*【订阅】无人机gps全局位置信息 */
      = nh.subscribe<sensor_msgs::NavSatFix>    /* 纬度、经度、高度，位置协方差，协方差类型*/
        (add2str(uavID, "mavros/global_position/global"), 10,
         &_FIXED_WING_SUB_PUB::global_position_form_px4_cb,
         &fixed_wing_sub_pub);
  
  fixed_wing_global_rel_alt_from_px4_sub        /*【订阅】无人机gps相对海拔（高度）*/
      = nh.subscribe<std_msgs::Float64>         /* 数据类型float64，自定义存储相对海拔 */
        (add2str(uavID, "mavros/global_position/rel_alt"), 10,
         &_FIXED_WING_SUB_PUB::fixed_wing_global_rel_alt_from_px4_cb,
         &fixed_wing_sub_pub);
  
  fixed_wing_umt_position_from_px4_sub          /* 【订阅】无人机里程计消息，自由空间中的位置和速度估计 */
      = nh.subscribe<nav_msgs::Odometry>        /* UTM坐标系*/
        (add2str(uavID, "mavros/global_position/local"), 10,
         &_FIXED_WING_SUB_PUB::umt_position_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_velocity_global_fused_from_px4_sub    /*【订阅】来自gps的速度输出*/
      = nh.subscribe<geometry_msgs::TwistStamped> /* 三向线速度和三向角速度 */
        (add2str(uavID, "mavros/global_position/raw/gps_vel"), 10,
         &_FIXED_WING_SUB_PUB::velocity_global_fused_from_px4_cb,
         &fixed_wing_sub_pub);

  fixed_wing_local_position_from_px4             /*【订阅】飞控的NED坐标系本地位置*/
      = nh.subscribe<geometry_msgs::PoseStamped> /*无人机ned，三方向点位置和四元数 */
        (add2str(uavID, "mavros/local_position/pose"), 10,
         &_FIXED_WING_SUB_PUB::local_position_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_velocity_ned_fused_from_px4_sub       /*【订阅】无人机ned飞控速度数据*/
      = nh.subscribe<geometry_msgs::TwistStamped> /* 三向线速度和角速度 */
        (add2str(uavID, "mavros/local_position/velocity_local"), 10,
         &_FIXED_WING_SUB_PUB::velocity_ned_fused_from_px4_cb,
         &fixed_wing_sub_pub);

  fixed_wing_acc_ned_from_px4_sub                             /*【订阅】无人机ned三向加速度*/
      = nh.subscribe<geometry_msgs::AccelWithCovarianceStamped> 
        (add2str(uavID, "mavros/local_position/accel"), 10,
         &_FIXED_WING_SUB_PUB::acc_ned_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_wind_estimate_from_px4_sub                          /*【订阅】无人机ned飞控风速估计 */
      = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped> /*三向加速度值和绕xyz轴旋转的角度 */
        (add2str(uavID, "mavros/wind_estimation"), 10,
         &_FIXED_WING_SUB_PUB::wind_estimate_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_battrey_state_from_px4_sub         /*订阅飞控系统电池状态消息*/
      = nh.subscribe<sensor_msgs::BatteryState> /*电压，电流，尚未使用的*/
        (add2str(uavID, "mavros/battery"), 10,
         &_FIXED_WING_SUB_PUB::battrey_state_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_waypoints_sub                      /*【订阅】无人机当前航点*/
      = nh.subscribe<mavros_msgs::WaypointList> /* 当前有效航点编号，航点列表*/
        (add2str(uavID, "mavros/mission/waypoints"), 10,
         &_FIXED_WING_SUB_PUB::waypointlist_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_waypointsreach_sub                     /*【订阅】无人机到达的航点*/
      = nh.subscribe<mavros_msgs::WaypointReached>  /* 抵达的航点的编号*/
        (add2str(uavID, "mavros/mission/reached"), 10,
         &_FIXED_WING_SUB_PUB::waypoints_reached_from_px4_cb,
         &fixed_wing_sub_pub);

  fixed_wing_altitude_from_px4_sub                  /*【订阅】高度信息*/
      = nh.subscribe<mavros_msgs::Altitude>         /* 单调的、amsl平均海平面、本地的、相对的、地形、底部间隙*/
        (add2str(uavID, "mavros/altitude"), 10,
         &_FIXED_WING_SUB_PUB::altitude_from_px4_cb, &fixed_wing_sub_pub);

  fixed_wing_air_ground_speed_from_px4_sub          /* 【订阅】目视飞行规则vfr，hud平视显示器消息*/
      = nh.subscribe<mavros_msgs::VFR_HUD>          /*空速、地速、航向(度)、油门、高度、爬升*/
        (add2str(uavID, "mavros/vfr_hud"), 10,
         &_FIXED_WING_SUB_PUB::air_ground_speed_from_px4_cb,
         &fixed_wing_sub_pub);



  fixed_wing_cmd_from_controller_sub                      /* 【订阅】【自定义】订阅来自上层控制器的四通道控制量 */ 
      = nh.subscribe<fixed_wing_formation_control::FWcmd> /* 固定翼控制指令的期望值 */
        (add2str(uavID,"fixed_wing_formation_control/fw_cmd"
        ), 10,
         &_FIXED_WING_SUB_PUB::cmd_from_controller_cb, &fixed_wing_sub_pub);


  //######################################## 订阅mavros消息+控制器指令cmd ###############################################//
  //
  //
  //
  //
  //######################################## 发布mavros消息+飞机状态states 给ROS##############################################//

  fixed_wing_local_pos_sp_pub = 
      nh.advertise<mavros_msgs::PositionTarget>(        /* 【发布】无人机本地位置、速度、加速度设定值*/
      add2str(uavID, "mavros/setpoint_raw/local"), 10); /* NED坐标系设置、控制遮盖mask flags、位置、速度、加速度或力、偏航角、偏航角速度*/

  fixed_wing_global_pos_sp_pub =
      nh.advertise<mavros_msgs::GlobalPositionTarget>(      /* 【发布】全球位置，速度和加速度设定值 */
          add2str(uavID, "mavros/setpoint_raw/global"), 10);/*坐标系设置、任务遮盖mask、纬度经度高度、速度、加速度或力、偏航角、偏航角速度*/

  fixed_wing_local_att_sp_pub = 
      nh.advertise<mavros_msgs::AttitudeTarget>(          /*【发布】姿态、角速度和油门设定值*/
      add2str(uavID, "mavros/setpoint_raw/attitude"), 10);/* NED坐标系设置、控制遮盖mask flags、位置、速度、加速度或力、偏航角、偏航角速度*/
  
  
  
  fixed_wing_states_pub = nh.advertise<fixed_wing_formation_control::FWstates>(/*【发布】【自定义】固定翼控制指令的期望值 */
          add2str(uavID,"fixed_wing_formation_control/fw_states"), 10);        //task_main订阅

  //##########################################发布消息###################################################//
  //
  //
  //
  //
  //##########################################服务###################################################//
  // 服务 修改系统模式
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(             /*【服务】设置飞控操作模式mode*/
      add2str(uavID, "mavros/set_mode"));

  arming_client = nh.serviceClient<mavros_msgs::CommandBool>(           /*【服务】切换命令常见的类型*/
      add2str(uavID, "mavros/cmd/arming"));                             /* 改变预备arming状态 */

  waypoint_setcurrent_client =
      nh.serviceClient<mavros_msgs::WaypointSetCurrent>(                /*【服务】请求设置当前航点waypoint*/
          add2str(uavID, "mavros/mission/set_current"));                /* 航点序号，成功标志位*/

  waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>(   /*【服务】从设备请求更新航点列表*/
      add2str(uavID, "mavros/mission/pull"));                           /* 成功状态、收到的航点*/

  waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>(   /*【服务】发送新的航路点表给设备*/
      add2str(uavID, "mavros/mission/push"));

  waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>( /*【服务】请求清空飞控的航点任务*/
      add2str(uavID, "mavros/mission/clear"));
  //##########################################服务###################################################//
  //
}
//***把订阅的所有消息组成包，发布出去
void PACK_FW_STATES::pack_fw_states() {
  /*###################################### 通用给结构体赋值；更新飞机状态 ############################################*/
  /* 通过设置传进来的无人机参数 */

  //控制模式
  fixed_wing_sub_pub.fw_states_form_mavros.control_mode =
      fixed_wing_sub_pub.current_state.mode;
  fixed_wing_sub_pub.fw_states_form_mavros.planeID = planeID;
  fixed_wing_sub_pub.fw_states_form_mavros.scurve = fixed_wing_sub_pub.cmd_from_controller.scurve;
  
  PACK_FW_STATES_INFO("planeID："<<planeID<<";"<<"scurve"<<fixed_wing_sub_pub.cmd_from_controller.scurve<<";");
  
  //以下为GPS信息
  fixed_wing_sub_pub.fw_states_form_mavros.altitude =
      fixed_wing_sub_pub.global_position_form_px4.altitude;       //***GPS高度获取不合适
  fixed_wing_sub_pub.fw_states_form_mavros.latitude =
      fixed_wing_sub_pub.global_position_form_px4.latitude;
  fixed_wing_sub_pub.fw_states_form_mavros.longitude =
      fixed_wing_sub_pub.global_position_form_px4.longitude;

  // GPS速度是在ned下的，
  fixed_wing_sub_pub.fw_states_form_mavros.global_vel_x =
      fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.y;
  fixed_wing_sub_pub.fw_states_form_mavros.global_vel_y =
      fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.x;
  fixed_wing_sub_pub.fw_states_form_mavros.global_vel_z =
      -fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.z;

  fixed_wing_sub_pub.fw_states_form_mavros.relative_alt =
      fixed_wing_sub_pub.global_rel_alt_from_px4.data;


  //以下为机体系和地面系的夹角，姿态角
  //***这样会影响角速度吗？角速度的正负还没确定怎么调
  fixed_wing_sub_pub.fw_states_form_mavros.roll_angle =
      fixed_wing_sub_pub.att_angle_Euler[0];
  fixed_wing_sub_pub.fw_states_form_mavros.pitch_angle =
      -fixed_wing_sub_pub.att_angle_Euler[1]; //添加负号转换到px4的系                             //姿态角是从IMU中获得四元数，然后转化为欧拉角
                                                                                                                                                                            //应该和local_position/pose一致

  if (-fixed_wing_sub_pub.att_angle_Euler[2] + deg_2_rad(90.0) > 0)
    fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle =
        -fixed_wing_sub_pub.att_angle_Euler[2] +
        deg_2_rad(90.0); //添加符号使增加方向相同，而且领先于px490°                    //****这里为什么让yaw领先90度未知
  else
    fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle =
        -fixed_wing_sub_pub.att_angle_Euler[2] + deg_2_rad(90.0) +
        deg_2_rad(360.0);

  //姿态四元数赋值
  att_angle[0] = fixed_wing_sub_pub.fw_states_form_mavros.roll_angle;
  att_angle[1] = fixed_wing_sub_pub.fw_states_form_mavros.pitch_angle;
  att_angle[2] = fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle;
  euler_2_quaternion(att_angle, att_quat);
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.w = att_quat[0];
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.x = att_quat[1];
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.y = att_quat[2];
  fixed_wing_sub_pub.fw_states_form_mavros.att_quater.z = att_quat[3];

  //以下为ned坐标系下的位置，速度
  fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_x =
      fixed_wing_sub_pub.local_position_from_px4.pose.position.y;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_y =
      fixed_wing_sub_pub.local_position_from_px4.pose.position.x;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_z =
      -fixed_wing_sub_pub.local_position_from_px4.pose.position.z;

  fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_x =
      fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.y;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_y =
      fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.x;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_z =
      -fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.z;

  //以下为体轴系加速度，体轴系当中的加速度是符合px4机体系的定义的
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc_x =
      fixed_wing_sub_pub.imu.linear_acceleration.x;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc_y =
      fixed_wing_sub_pub.imu.linear_acceleration.y;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc_z =
      fixed_wing_sub_pub.imu.linear_acceleration.z;

  fixed_wing_sub_pub.fw_states_form_mavros.body_acc.x =
      fixed_wing_sub_pub.imu.linear_acceleration.x;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc.y =
      fixed_wing_sub_pub.imu.linear_acceleration.y;
  fixed_wing_sub_pub.fw_states_form_mavros.body_acc.z =
      fixed_wing_sub_pub.imu.linear_acceleration.z;

  //以下来自altitude
  fixed_wing_sub_pub.fw_states_form_mavros.relative_hight =
      fixed_wing_sub_pub.altitude_from_px4.relative;
  fixed_wing_sub_pub.fw_states_form_mavros.ned_altitude =
      fixed_wing_sub_pub.altitude_from_px4.local;

  //空速和地速
  fixed_wing_sub_pub.fw_states_form_mavros.air_speed =
      fixed_wing_sub_pub.air_ground_speed_from_px4.airspeed;
  fixed_wing_sub_pub.fw_states_form_mavros.ground_speed =
      fixed_wing_sub_pub.air_ground_speed_from_px4.groundspeed;

  //风估计
  fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_x =
      fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.y;
  fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_y =
      fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.x;
  fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_z =
      -fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.z;
  //电池状态
  fixed_wing_sub_pub.fw_states_form_mavros.battery_current =
      fixed_wing_sub_pub.battrey_state_from_px4.current;
  fixed_wing_sub_pub.fw_states_form_mavros.battery_precentage =
      fixed_wing_sub_pub.battrey_state_from_px4.percentage;
  fixed_wing_sub_pub.fw_states_form_mavros.battery_voltage =
      fixed_wing_sub_pub.battrey_state_from_px4.voltage;

    //角速度
    //***参照角度赋值部分的正负号添加形式 感觉没问题 看看发布的话题FWstates的正负
    fixed_wing_sub_pub.fw_states_form_mavros.pitch_rate = -fixed_wing_sub_pub.imu.angular_velocity.y;
    fixed_wing_sub_pub.fw_states_form_mavros.roll_rate = fixed_wing_sub_pub.imu.angular_velocity.x;
    fixed_wing_sub_pub.fw_states_form_mavros.yaw_rate = -fixed_wing_sub_pub.imu.angular_velocity.z;

  //把订阅的所有消息组成包，发布出去；打包的mavros发布
  fixed_wing_states_pub.publish(fixed_wing_sub_pub.fw_states_form_mavros);
  
}

/* 把需要发送给PX4的控制量发送给MAVROS */
void PACK_FW_STATES::msg_to_mavros() {
  //将期望值转换一下坐标系，并转化为四元数
  float angle[3], quat[4];

  angle[0] = fixed_wing_sub_pub.cmd_from_controller.roll_angle_sp; /* 滚转角 */
  angle[1] = -fixed_wing_sub_pub.cmd_from_controller.pitch_angle_sp; //***感觉这里的yaw一直是定值，虽然yaw不影响px4的姿态内环，但是应该会影响四元数的转化吧
  angle[2] =
      -fixed_wing_sub_pub.cmd_from_controller.yaw_angle_sp + deg_2_rad(90.0);

//***试一下yaw改为当前状态的yaw
// angle[2] = fixed_wing_sub_pub.att_angle_Euler[2];  //***此时无变化
  euler_2_quaternion(angle, quat);
  cout << "roll:" << fixed_wing_sub_pub.cmd_from_controller.roll_angle_sp << endl;
  cout << "pitch:" << fixed_wing_sub_pub.cmd_from_controller.pitch_angle_sp << endl;
  cout << "yaw:" << angle[2] << endl;
  cout<<"yaw_e:"<<fixed_wing_sub_pub.cmd_from_controller.yaw_angle_sp<<endl;
  cout<<"throttle:"<<fixed_wing_sub_pub.cmd_from_controller.throttle_sp<<endl;   //***看看发送的油门有没有起作用
  fixed_wing_sub_pub.att_sp.type_mask =
      7; // 1+2+4+64+128 body.rate_x,body.rate_y,body.rate_z thrust..
  fixed_wing_sub_pub.att_sp.orientation.w = quat[0];
  fixed_wing_sub_pub.att_sp.orientation.x = quat[1];
  fixed_wing_sub_pub.att_sp.orientation.y = quat[2];
  fixed_wing_sub_pub.att_sp.orientation.z = quat[3];
  fixed_wing_sub_pub.att_sp.thrust =
      fixed_wing_sub_pub.cmd_from_controller.throttle_sp;
  fixed_wing_local_att_sp_pub.publish(fixed_wing_sub_pub.att_sp); /*向MAVROS发布姿态、四元数、角速度和油门设定值*/
}



void PACK_FW_STATES::srv_to_mavros() {
  //切换飞行模式，设置模式
  fixed_wing_sub_pub.mode_cmd.request.custom_mode =
      fixed_wing_sub_pub.cmd_from_controller.cmd_mode;

    if (set_mode_client.call(fixed_wing_sub_pub.mode_cmd)) /* 请求SetMode类型的服务*/        
    {
        PACK_FW_STATES_INFO("设置模式：:"<<fixed_wing_sub_pub.mode_cmd.request.custom_mode);/* 请求成功，打印当前模式*/ 
    }     
}

void PACK_FW_STATES::run(int argc, char **argv) {
  ros::Rate rate(50.0);

  ros_sub_and_pub(); //ROS初始化声明
  PACK_FW_STATES_INFO("打包开始");

  long begin_time = get_sys_time();

  while (ros::ok()) {

    print_counter++; //打印提示

    if (print_counter >= 100) {
      PACK_FW_STATES_INFO("本机ID:" << planeID);
      PACK_FW_STATES_INFO(
          "历时:" << fixed << setprecision(5)
                  << double(get_time_from_begin(begin_time)) / 1000 << " 秒");
      print_counter = 0;
    }

    pack_fw_states();//发布出去，然后task_main订阅
    

    msg_to_mavros();//把需要发送给PX4的控制量发送给MAVROS
    srv_to_mavros();//把切换模式的服务发送给MAVROS
    PACK_FW_STATES_INFO("无人机planeID: " << planeID);    //***三架长机都这样
       

    /*else   
    {
        msg_to_mavros();
        srv_to_mavros();   
        PACK_FW_STATES_INFO("僚机ID: " << planeID);   
        
    }*/

    ros::spinOnce(); //挂起一段时间，保证周期的速度,执行ROS后退出，执行下一次的while循环，没有的话只会在里面订阅数据；
    rate.sleep(); //多长时间执行一次
  }
  PACK_FW_STATES_INFO("打包退出");
}
