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

   int planeID{1};  //***设置飞机ID 用来确定编队的期望点

    /* 一系列的飞机动力学模型参数 */
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
        float max_arispd_sp{22.0}; //***TECS给的速度期望范围在3-30，这里这样设置符合  QGC的vtol是25 试一下大点的速度(原为20)

        /* 飞机空速最小设定值 */
        float min_arispd_sp{12.0};   //***根据QGC采取的飞机模型STALL_SPEED为10
    };


    /* 领机状态信息 */
    struct _s_leader_states
    {
        /* 姿态只有姿态那个控制器才可能用到 */
        float pitch_angle{0};

        float yaw_angle{0};

        float roll_angle{0};

        /* 有时候不能直接得到领机的航向信息 */
        bool yaw_valid{false};

        /* NED速度与GPS速度是不一致的   ？区别在哪里？在计算中只使用了GPS速度信息*/  
        float ned_vel_x{0};

        float ned_vel_y{0};

        float ned_vel_z{0};

        /* GPS速度 */
        float global_vel_x{0};

        float global_vel_y{0};

        float global_vel_z{0};

        double latitude{0};

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float wind_estimate_x{0};

        float wind_estimate_y{0};

        float wind_estimate_z{0};

        /*角速度，imu获得*/
        float pitch_rate{0};

        float roll_rate{0};

        float yaw_rate{0};
    };

    /* 本机状态信息 */
    struct _s_fw_states
    {
        string flight_mode;

        float pitch_angle{0};

        float yaw_angle{0};

        float roll_angle{0};

        float att_quat[4];

        /* NED速度 */
        float ned_vel_x{0};

        float ned_vel_y{0};

        float ned_vel_z{0};

        /* GPS速度 */
        float global_vel_x{0};

        float global_vel_y{0};

        float global_vel_z{0};

        float body_acc[3];

        /*角速度 imu获得*/
        float pitch_rate{0};

        float roll_rate{0};

        float yaw_rate{0};

        /* 内部计算后填充，或者外部填充均可 */
        float ned_acc[3];

        float rotmat[3][3];

        double latitude{0};

        double longitude{0};

        double altitude{0};

        float relative_alt{0};

        float air_speed{0};

        float wind_estimate_x{0};

        float wind_estimate_y{0};

        float wind_estimate_z{0};

        bool in_air{true};

        bool altitude_lock{false};

        /* TODO:添加yaw_valid的判断，因为此是一个十分重要的计算量，将来的控制量基本与之有关 */
        bool yaw_valid{true};
    };

    /* 编队队形几何偏移 */
    //***根据需要选择，有冗余
    //***注意，这里的备注只是一个冗余选择，实际实现的时候只使用了在全局坐标系下的误差，其他部分可能会有歧义的部分，以全局误差为主
    struct _s_formation_offset
    {
        /* 机体系 */
        float xb{0};
        float yb{0};
        float zb{0};

        /* 航迹系 */
        float xk{0};
        float yk{0};
        float zk{0};

        /* NED系 */
        float ned_n{0};
        float ned_e{0};
        float ned_d{0};
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
    };

    /* 本机误差，包括与领机的偏差 */
    struct _s_fw_error
    {
        /* ned坐标系之下的位置误差 */
        float P_N{0};
        float P_E{0};
        float P_D{0};
        float P_NE{0};

        /* 体轴系位置误差<与自己期望> */
        float PXb{0};
        float PYb{0};
        float PZb{0};

        /* 航迹轴系位置误差<与自己期望> */
        float PXk{0};
        float PYk{0};
        float PZk{0};

        /* 体轴系速度误差<与自己期望> */
        float VXb{0};
        float VYb{0};
        float VZb{0};
        float Vb{0};

        /* 航迹轴速度误差<与自己期望> */
        float VXk{0};
        float VYk{0};
        float VZk{0};
        float Vk{0};

        /* 体轴系速度误差<与领机> */
        float led_fol_vxb{0};
        float led_fol_vyb{0};
        float led_fol_vzb{0};
        float led_fol_vb{0};

        /* 航迹轴系速度误差<与领机> */
        float led_fol_vxk{0};
        float led_fol_vyk{0};
        float led_fol_vzk{0};
        float led_fol_vk{0};

        /* 航迹轴系下的速度角度（方向）误差 */
        //***可当作航向角误差
        float led_fol_eta{0};

    };

    /* 四通道控制量 */
    struct _s_4cmd
    {
        float roll{0};
        float pitch{0};
        float yaw{0};                //***这里默认yaw为0 后续发布消息的时候也没有多余的处理，这样会影响后边欧拉角转化为四元数指令吗？
        float thrust{0};
    };

    /**
    * 控制器初始化、设置函数（组）
    */

    /* 更新领从机状态*/
    void update_led_fol_states(const struct _s_leader_states *leaderstates,
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
    /* 控制时间间隔 */
    float _dt{0.02};
    /* 控制时间间隔max */
    float _dtMax{0.1};
    /* 控制时间间隔min */
    float _dtMin{0.01};

    /* 编队队形偏移量 */
    _s_formation_offset formation_offset;
    /* 飞机模型参数 */
    _s_fw_model_params fw_params;

    /* 领机状态 */
    _s_leader_states leader_states;
    /* 从机状态 */
    _s_fw_states fw_states;

    /* 领机正在飞行标志位 */
    bool led_in_fly{false};
    /* 从机正在飞行标志位 */
    bool fol_in_fly{false};

    /* 最后的控制量 */
    _s_4cmd _cmd;
    /* 本机的期望 */
    _s_fw_sp fw_sp;
    /* 本机误差，包括与期望的差和领机的偏差 */
    _s_fw_error fw_error;

private:
};

#endif
