/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#include "switch_fw_mode.hpp"

/**
 * @Input: int
 * @Output:
 * @Description: 设定当前飞机的ID
 */
void SWITCH_FW_MODE::set_planeID(int id) {
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

void SWITCH_FW_MODE::run() {

  begin_time = ros::Time::now(); /* 记录启控时间 */
  ros::Rate rate(5.0);           /*  频率 [5Hz] */

  /* 【订阅】无人机当前状态 */
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      add2str(uavID, "mavros/state"), 10, &SWITCH_FW_MODE::state_cb, this);

  /* 【发布】编队控制器期望状态 */
  ros::Publisher task_cmd_mode_pub =
      nh.advertise<fixed_wing_formation_control::Fw_cmd_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_cmd_mode"), 10);

  /* 【服务】 修改锁定状态 */
  ros::ServiceClient arm_client =
      nh.serviceClient<mavros_msgs::CommandBool>(add2str(uavID, "mavros/cmd/arming"));

  /* 【服务】 修改系统模式 */
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>(add2str(uavID, "mavros/set_mode"));

  // ros::Publisher leader_position_sp_pub = 
  // nh.advertise<mavros_msgs::PositionTarget>(add2str(uavID, "mavros/setponit_raw/local"),10);

  while (ros::ok() && (!outflag)) {

    float current_time = get_ros_time(begin_time); /* 当前时间 */

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>SWITCH_FW_MODE<<<<<<<<<<<<<<<<<<<<<<<<<<"
         << endl;
    SWITCH_FW_MODE_INFO("本机ID:"<<planeID);
    SWITCH_FW_MODE_INFO("当前时刻："<<current_time);
    SWITCH_FW_MODE_INFO("当前模式 : [ " << current_state.mode << " ]");
    SWITCH_FW_MODE_INFO("更换模式：1 offboard, 2 mission, 3 out" );
    SWITCH_FW_MODE_INFO("第二个参数为切换编队：1 竖排南北一字, 2 三角，3 横排东西一字" );

    int mode_type = 0;    /* 模式类型 */
    cin >> mode_type;

    int formation_type = 0;    //***这样切换队形的时候，连模式也要一并发送，不过保持不变就行吧
    int formation_type_pre = formation_type;  //***储存之前的编队类型状态
    cin >> formation_type;

    if (mode_type == 1) {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_formation = true;
      task_cmd_mode.formation_type = formation_type;
     //***
    } else if (mode_type == 2) {
      mode_cmd.request.custom_mode = "AUTO.MISSION";
    } else if (mode_type == 3) {
      outflag = true;
      break;
    }
    /*  如果当前模式与设定模式不一致，切换，（有次数限制） */
    //***编队模式也应当考虑进来
    while (((mode_cmd.request.custom_mode != current_state.mode) ||  (formation_type_pre != formation_type)) &&
           (counters <= times_out) ) {

      counters++;
      if(set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent)         //***在这里就完成了任务模式的切换, 领机的切换却不行  判断一下有没有成功调用服务
      {                                                                                                                                                               //***参照PX4官网的offboard模式切换
        SWITCH_FW_MODE_INFO("current_state:" << current_state.mode);
        SWITCH_FW_MODE_INFO("sp_state:" << mode_cmd.request.custom_mode);
        SWITCH_FW_MODE_INFO(" 当前队形: " << task_cmd_mode.formation_type);
        task_cmd_mode_pub.publish(task_cmd_mode);  //***切换成功发送整体控制量  注释调也可以切换，为什么跟随不了？
      }     
    }

    ros::spinOnce(); /*  挂起一段时间，等待切换结果 */
    rate.sleep();

    counters = 0;
  }
}

float SWITCH_FW_MODE::get_ros_time(ros::Time begin) {
  ros::Time time_now = ros::Time::now();
  float currTimeSec = time_now.sec - begin.sec;
  float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimeSec + currTimenSec);
}
