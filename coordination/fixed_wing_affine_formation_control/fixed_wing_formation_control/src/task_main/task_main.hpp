/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */
//***建议参数的读取放在外部文件中，这样便于修改，不过这里直接结构体赋值了

#ifndef _TASK_MAIN_HPP_
#define _TASK_MAIN_HPP_

/* 把被包含文件的全部内容输入到源文件#include指令所在的位置 */
#include <ros/ros.h>          /* 添加ROSC++客户端库必须包含的头文件 */
#include <iostream>           /* 添加输入输出流文件	*/
#include "../fixed_wing_lib/syslib.hpp"                            /* 添加时间戳文件	*/
#include "fixed_wing_formation_control/FWstates.h"                 /* 固定翼的编队控制，自定义可能用到的消息类型 */
#include "fixed_wing_formation_control/FWcmd.h"                    /* 固定翼控制指令的期望值文件 */
#include "fixed_wing_formation_control/Formation_control_states.h" /* 编队控制量，编队经纬度/编队误差	*/
#include "fixed_wing_formation_control/Fwmonitor.h"                /* 飞机状态检测标志位 */
#include "fixed_wing_formation_control/Fw_cmd_mode.h"              /* 控制飞行模式状态量 */
#include "fixed_wing_formation_control/Fw_current_mode.h"          /* 飞机当前控制模式赋值 */
#include "../formation_controller/formation_controller.hpp"        /* 导入飞机编队控制程序母本 */
#include "../formation_controller/abs_formation_controller.hpp"    /* 导入飞机编队控制程序继承者 */

/* 命名空间：定义标识符的各种可见范围*/
using namespace std;

/* 定义打印输出语句 */
#define TASK_MAIN_INFO(a) cout << "[TASK_MAIN_INFO]:" << a << endl

class TASK_MAIN
{
private:
    int planeID{1};             /*飞机编号*/

    float scurve{0};            /* 虚拟目标点弧长*/

    int numUAV{4};              /* 无人机总数 */

    int numLeader{2};           /* 长机总数*/
    
    int formation_type_id{0};   /* 队形切换 */

    float itsneighbor_scurve0={0.0};/* 长机的邻居弧长参数 */
    float itsneighbor_scurve1={0.0};/* 长机的邻居弧长参数 */

    string uavID{ "uav1/"};     /* 僚机编号，用于命名空间 */
    string neighborID[2]={ "uav1/","uav2/"}; /*长机的邻居*/    
    string leaderID{ "uav1/"};     /* 僚机编号，用于命名空间 */
    string everyuavID[6]={"uav0/","uav1/","uav2/","uav3/","uav4/","uav5/"};/*所有无人机编号*/

    void print_data(const struct ABS_FORMATION_CONTROLLER ::_s_fw_states *p); /*打印滤波后僚机数据*/

    ros::NodeHandle nh;     /*ros句柄，nh在<node_name>下*/
    ros::Time begin_time;   /* 定义起始时间 */
    /* 当前时间 */
    float current_time;   

    /*获取当前时间，返回当前相对于起始时间的时间差  */
    float get_ros_time(ros::Time begin); 

    /* 创建ros_sub_pub函数中订阅/发布的对象，用于存储订阅的信息 */
    ros::Subscriber fwmonitor_sub;               /*【订阅】监控节点飞机以及任务状态，来自fw_monitor任务状态的flags*/
    ros::Subscriber fw_states_sub;               /*【订阅】固定翼全部状态量,这里订阅了来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber every_fw_states_sub0;      /*【订阅】所有固定翼无人机全部状态量,这里订阅了来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber every_fw_states_sub1;      /*【订阅】所有固定翼无人机全部状态量,这里订阅了来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber every_fw_states_sub2;      /*【订阅】所有固定翼无人机全部状态量,这里订阅了来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber every_fw_states_sub3;      /*【订阅】所有固定翼无人机全部状态量,这里订阅了来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber every_fw_states_sub4;      /*【订阅】所有固定翼无人机全部状态量,这里订阅了来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber every_fw_states_sub5;      /*【订阅】所有固定翼无人机全部状态量,这里订阅了来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber neighbor_sub0;            /*【订阅】订阅长机的邻居长机信息状态量 ，来自通讯节点，或者视觉节点长机全部状态量*/
    ros::Subscriber neighbor_sub1;            /*【订阅】订阅长机的邻居长机信息状态量 ，来自通讯节点，或者视觉节点长机全部状态量*/
    ros::Subscriber leader_states_sub;           /*【订阅】订阅僚机的所有长机信息状态量，来自通讯节点，或者视觉节点长机全部状态量*/
    ros::Subscriber fw_cmd_mode_sub;             /*【订阅】来自commander状态机的控制指令，指定比赛模式*/
    ros::Publisher formation_control_states_pub; /*【发布】编队控制器状态，只起到一个监控作用*/
    ros::Publisher fw_cmd_pub;                   /*【发布】飞机四通道控制量，控制指令期望值,三个欧拉角和油门,scurve*/
    ros::Publisher fw_current_mode_pub;          /*【发布】飞机当前所处任务阶段,飞机当前控制模式赋值*/

    fixed_wing_formation_control::FWstates fwstates;                                 /*自定义FWstates--------------飞机打包的全部状态*/
    fixed_wing_formation_control::FWstates everyfwstates0;                                 /*自定义FWstates--------------所有长机打包的全部状态*/
    fixed_wing_formation_control::FWstates everyfwstates1;                                 /*自定义FWstates--------------所有长机打包的全部状态*/
    fixed_wing_formation_control::FWstates everyfwstates2;                                 /*自定义FWstates--------------所有长机打包的全部状态*/
    fixed_wing_formation_control::FWstates everyfwstates3;                                 /*自定义FWstates--------------所有长机打包的全部状态*/
    fixed_wing_formation_control::FWstates everyfwstates4;                                 /*自定义FWstates--------------所有长机打包的全部状态*/
    fixed_wing_formation_control::FWstates everyfwstates5;                                 /*自定义FWstates--------------所有长机打包的全部状态*/
    fixed_wing_formation_control::FWcmd fw_4cmd;                                     /*自定义FWcmd-------------------飞机四通道控制量*/
    fixed_wing_formation_control::FWcmd neighbor0;                                    /*自定义FWcmd--------------长机的邻居的scurve状态*/
    fixed_wing_formation_control::FWcmd neighbor1;                                    /*自定义FWcmd--------------长机的邻居的scurve状态*/
    fixed_wing_formation_control::FWstates leaderstates;                                /*自定义FWstates--------------飞机打包的全部状态*/
    fixed_wing_formation_control::Formation_control_states formation_control_states; /*自定义Formation_control_states--编队控制状态量*/
    fixed_wing_formation_control::Fwmonitor fwmonitor_flag;                          /*自定义Fwmonitor----------------任务状态的flags*/
    fixed_wing_formation_control::Fw_cmd_mode fw_cmd_mode;                           /*自定义Fw_cmd_mode--来自commander的命令飞机模式*/
    fixed_wing_formation_control::Fw_current_mode fw_current_mode;                   /*自定义Fw_current_mode--要发布的飞机当前任务模式*/
    
    /* ros的订阅发布声明函数 */
    void ros_sub_pub();           
    /* ros的订阅声明函数,订阅长机邻居或僚机的长机信息 */               

    /* 回调函数 */
    /*任务状态flagscallback*/
    void fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg);     

    /*僚机状态callback*/
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 

    /*所有无人机状态callback*/
    void fw_state_cb0(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 
    
    /*所有无人机状态callback*/
    void fw_state_cb1(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 

    /*所有无人机状态callback*/
    void fw_state_cb2(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 

    /*所有无人机状态callback*/
    void fw_state_cb3(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 

    /*所有无人机状态callback*/
    void fw_state_cb4(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 

    /*所有无人机状态callback*/
    void fw_state_cb5(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 

    /*长机状态callback*/         
    void neighbor_cb0(const fixed_wing_formation_control::FWcmd::ConstPtr &msg); 
    
    /*长机状态callback*/         
    void neighbor_cb1(const fixed_wing_formation_control::FWcmd::ConstPtr &msg); 
    
    /*长机状态callback*/         
    void leader_states_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg); 


    /*commander指令callback*/
    void fw_cmd_mode_cb(const fixed_wing_formation_control::Fw_cmd_mode::ConstPtr &msg);    

    /*编队控制主函数，完成对于领机从机状态的赋值，传入编队控制器 */
    void control_formation();  

    void follower_control_formation();

    ABS_FORMATION_CONTROLLER formation_controller; /*调用abs编队控制器，在本cpp中命名为formation_controller*/
    string fw_col_mode_current{"MANUAL"};   /*当前模式*/
    string fw_col_mode_last{"MANUAL"};      /*上一时刻模式*/
    
    struct ABS_FORMATION_CONTROLLER::_s_fw_model_params fw_params;         /*飞机模型参数*/
    struct ABS_FORMATION_CONTROLLER::_s_tecs_params tecs_params;           /*编队控制器内部TECS控制器参数*/

    struct ABS_FORMATION_CONTROLLER::_s_fw_states thisfw_states;           /*僚机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states leader_states;        /*僚机的长机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states everyfw_states0;        /*所有无人机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states everyfw_states1;        /*所有无人机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states everyfw_states2;        /*所有无人机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states everyfw_states3;        /*所有无人机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states everyfw_states4;        /*所有无人机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states everyfw_states5;        /*所有无人机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_4cmd formation_cmd;                /*四通道控制量*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_error formation_error;          /*编队误差以及偏差*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_sp formation_sp;                /*编队控制运动学期望值*/
    

    /* 编队状态值赋值发布，编队位置误差（机体系和ned），速度误差以及期望空速，gps，期望地速 */
    void formation_states_pub();            
    

public:

  /* 主循环，在main中调用 */
  void run();
  /* 设定当前无人机的ID */
  void set_planeID(int id);


};

#endif
