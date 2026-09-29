/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#ifndef _FORMATION_CONTROLLER_HPP_
#define _FORMATION_CONTROLLER_HPP_

#include <iostream>
#include "../fixed_wing_lib/mathlib.hpp"

using namespace std;

#ifndef the_space_between_lines
#define the_space_between_lines 1  /* 为了打印中间空格 */
#define the_space_between_blocks 3 /* 为了打印中间空格 */
#define FORMATION_CONTROLLER_INFO(a) cout << "[FORMATION_CONTROLLER_INFO]:" << a << endl
#endif

class FORMATION_CONTROLLER
{
public:
    /**
    * 控制器重要的结构体，承担着数据载体与容器的作用、
    * 将控制器内部的数据规整，方便传递与维护
    * 十分重要的数据桥梁，写成public为了外部访问结构体的声明
    */

   int planeID{0};  //***设置飞机ID 用来确定编队的期望点

   int itsleaderID{0};         /* 无人机僚机的长机号 */
   float currenttime{0};
   float itsneighbor_scurve0={0.0};/* 长机的邻居弧长参数 */
   float itsneighbor_scurve1={0.0};/* 长机的邻居弧长参数 */
    /* 飞机的动力学模型参数 */
    struct _s_fw_model_params
    {
        /*for tecs use*/

        /* 最小油门 */
        float throttle_min{0.1};

        /* 最大油门 */
        float throttle_max{1};

        /* 巡航油门 */
        float throttle_cruise{0.3};

        /* 最小俯仰角rad */
        float pitch_min_rad{-PI /4};   //设置较小的失速度迎角 10度  PI/36  18  根据QGC的门限设为45度

        /* 最大俯仰角rad */
        float pitch_max_rad{PI / 4};

        /* 最大俯仰角速度 */
        float pitch_rate_max{PI / 3};

        /* for later_controller use*/

        /* 最大滚转角rad */
        float roll_max{PI / 4}; 

        /* 最大滚转角速度rad/s */
        float roll_rate_max{PI / 3}; 

        /*for generate the airspeed setpoint use*/

        /* 飞机前向最大瞬时加速度 */                   /*这个参数去QGC里边看看，能不能对应上*/
        float maxinc_acc{20.0};

        /* 飞机减速最大瞬时加速度 */
        float maxdec_acc{10.0};

        /* 飞机空速最大设定值,此处的最大速度，一定要和飞机的最快速度贴合，否则容易造成油门抖动 */
        float vmax{22.0}; //***TECS给的速度期望范围在3-30，这里这样设置符合  QGC的vtol是25 试一下大点的速度(原为20)

        /* 飞机空速最小设定值 */
        float vmin{10.0};   //***根据QGC采取的飞机模型STALL_SPEED为10

        /* 无人机最大偏航角速度 */
        float omegamax{0.4};
        /* 无人机最小偏航角速度，反向*/
        float omegamin{-0.4};
    };


    /* 僚机状态信息 */
    struct _s_fw_states
    {
        string flight_mode;//飞行模式

        int planeID{0};

        float scurve{0};//虚拟目标点弧长

        float pitch_angle{0};//俯仰角

        float yaw_angle{0};//偏航角

        float roll_angle{0};//滚转角

        float att_quat[4];//四元数

        /* NED速度 */
        float ned_vel_x{0};/* NED x方向速度 */  

        float ned_vel_y{0};/* NED y方向速度 */  

        float ned_vel_z{0};/* NED z方向速度 */  

        /* GPS速度 */
        float global_vel_x{0};/* GPS x方向速度 */

        float global_vel_y{0};/* GPS y方向速度 */

        float global_vel_z{0};/* GPS z方向速度 */

        float body_acc[3];

        /*角速度 imu获得*/
        float pitch_rate{0};/*俯仰角速度，imu获得*/

        float roll_rate{0};/*滚转角速度，imu获得*/

        float yaw_rate{0};/*偏航角速度，imu获得*/

        /* 内部计算后填充，或者外部填充均可 */
        float ned_acc[3];

        float rotmat[3][3];/* 旋转矩阵 */

        double latitude{0};/* 纬度 */

        double longitude{0};/* 经度 */

        double altitude{0};/* 高度 */

        float relative_alt{0};

        float air_speed{0};/* 空速 */

        bool in_air{true};/* 起飞标志位 */

        bool altitude_lock{false};/* 保证TECS标志位 */

        /* TODO:添加yaw_valid的判断，因为此是一个十分重要的计算量，将来的控制量基本与之有关 */
        bool yaw_valid{true};
    };

    /* 编队队形几何偏移 */
    //***根据需要选择，有冗余
    //***注意，这里的备注只是一个冗余选择，实际实现的时候只使用了在全局坐标系下的误差，其他部分可能会有歧义的部分，以全局误差为主
    struct _s_formation_offset
    {
        /* 机体系 */
        float xb{0};/* 机体系xb */
        float yb{0};/* 机体系yb */
        float zb{0};/* 机体系zb */

        /* 航迹系 */
        float xk{0};/* 航迹系xk */
        float yk{0};/* 航迹系yk */
        float zk{0};/* 航迹系zk */

        /* NED系 */
        float ned_n{0};/* NED系北 */
        float ned_e{0};/* NED系东 */
        float ned_d{0};/* NED系地 */
    };

    /* 这个结构体为了区分，角度以及油门的期望值就是单独要发布的，是由运动学位置以及速度的期望值以及当前飞机的状态，是计算出来的。 */
    struct _s_fw_sp
    {
        float ned_vel_x{0};

        float ned_vel_y{0};

        float ned_vel_z{0};

        double latitude{0};

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float ground_speed{0};
        
        float scurve{0};
        
        int planeID{0};
    };

    /* 本机误差，包括与领机的偏差 */
    struct _s_fw_error
    {
        int planeID{0};
        /* ned坐标系之下的位置误差 */
        float P_N{0};/* ned坐标系之下北的位置误差 */
        float P_E{0};/* ned坐标系之下东的位置误差 */
        float P_D{0};/* ned坐标系之下地的位置误差 */
        float P_NE{0};

        /* 体轴系位置误差<与自己期望> */
        float PXb{0};/* 体轴系x位置误差<与自己期望> */
        float PYb{0};/* 体轴系y位置误差<与自己期望> */
        float PZb{0};/* 体轴系z位置误差<与自己期望> */

        /* 航迹轴系位置误差<与自己期望> */
        float PXk{0};/* 航迹轴系x位置误差<与自己期望> */
        float PYk{0};/* 航迹轴系y位置误差<与自己期望> */
        float PZk{0};/* 航迹轴系z位置误差<与自己期望> */

        /* 体轴系速度误差<与自己期望> */
        float VXb{0};/* 体轴系x方向速度误差<与自己期望> */
        float VYb{0};/* 体轴系y方向速度误差<与自己期望> */
        float VZb{0};/* 体轴系z方向速度误差<与自己期望> */
        float Vb{0};/* 体轴系速度误差<与自己期望> */

        /* 航迹轴速度误差<与自己期望> */
        float VXk{0};/* 航迹轴x方向速度误差<与自己期望> */
        float VYk{0};/* 航迹轴y方向速度误差<与自己期望> */
        float VZk{0};/* 航迹轴z方向速度误差<与自己期望> */
        float Vk{0};/* 航迹轴速度误差<与自己期望> */

        /* 体轴系速度误差<与领机> */
        float led_fol_vxb{0};/* 体轴系x方向速度误差<与领机> */
        float led_fol_vyb{0};/* 体轴系y方向速度误差<与领机> */
        float led_fol_vzb{0};/* 体轴系z方向速度误差<与领机> */
        float led_fol_vb{0};/* 体轴系速度误差<与领机> */

        /* 航迹轴系速度误差<与领机> */
        float led_fol_vxk{0};/* 航迹轴系x方向速度误差<与领机> */
        float led_fol_vyk{0};/* 航迹轴系y方向速度误差<与领机> */
        float led_fol_vzk{0};/* 航迹轴系z方向速度误差<与领机> */
        float led_fol_vk{0};/* 航迹轴系速度误差<与领机> */

        /* 航迹轴系下的速度角度（方向）误差，可当作航向角误差 */
        float led_fol_eta{0};

    };

    /* 四通道控制量 */
    struct _s_4cmd
    {
        float roll{0};  /* 滚转角 */
        float pitch{0}; /* 俯仰角 */
        float yaw{0};   /* 偏航角--->这里默认yaw为0 后续发布消息的时候也没有多余的处理，这样会影响后边欧拉角转化为四元数指令吗？*/
        float thrust{0};/* 油门 */
        float scurve{0};/* 虚拟目标点弧长 */
        int planeID{0};

    };

    /**
    * 控制器初始化、设置函数（组）
    */

    /* 更新长、僚机状态*/
    void update_everyfw_states(const struct _s_fw_states *everyfw_states0,
                               const struct _s_fw_states *everyfw_states1,
                               const struct _s_fw_states *everyfw_states2,
                               const struct _s_fw_states *everyfw_states3,
                               const struct _s_fw_states *everyfw_states4,
                               const struct _s_fw_states *everyfw_states5,
                               const struct _s_fw_states *thisfw_states);
    /*更新长机状态*/
    void update_leaders_states(float current_time,
                               const float *itsneighborstates0,
                               const float *itsneighborstates1,
                               const struct _s_fw_states *thisfw_states);
    /* 设定编队形状 */
    void set_formation_type(int formation_type);

    /* 设定飞机模型参数 */
    void set_fw_model_params(struct _s_fw_model_params &input_params);

    /* 领机从机起飞识别函数 */
    bool identify_led_fol_states();

    /**
    * 控制输出获取函数（组）
    */

    /* 得到编队控制后的四通道控制量 */
    void get_formation_4cmd(struct _s_4cmd &fw_cmd);

    /* 得到编队中本机的运动学期望值 */
    void get_formation_sp(struct _s_fw_sp &formation_sp);

    /* 得到编队控制误差 */
    void get_formation_error(struct _s_fw_error &formation_error);

    //***设定从机ID
    void set_fw_planeID(int id);

  protected:
    
    float _dt{0.02};/* 控制时间间隔 */
    
    float _dtMax{0.1};/* 控制时间间隔max */
    
    float _dtMin{0.01};/* 控制时间间隔min */

    
    _s_formation_offset formation_offset;/* 编队队形偏移量 */
    
    _s_fw_model_params fw_params;/* 飞机模型参数 */

    _s_fw_states leaderstates;/* 僚机的长机状态 */

    _s_fw_states everyfwstates0;/*每架无人机状态*/
    _s_fw_states everyfwstates1;/*每架无人机状态*/
    _s_fw_states everyfwstates2;/*每架无人机状态*/
    _s_fw_states everyfwstates3;/*每架无人机状态*/
    _s_fw_states everyfwstates4;/*每架无人机状态*/
    _s_fw_states everyfwstates5;/*每架无人机状态*/
    

    _s_fw_states fw_states;/* 本机状态 */

    
    bool led_in_fly{false};/* 领机正在飞行标志位 */
    
    bool fol_in_fly{false};/* 从机正在飞行标志位 */

    
    _s_4cmd _cmd;/* 最后的控制量 */
    
    _s_fw_sp fw_sp;/* 长机的期望位置 */
    
    _s_fw_error fw_error;/* 长机误差，包括与期望的偏差 */

private:
};

#endif
