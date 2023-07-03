/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */
#ifndef _ABS_FORMATION_CONTROLLER_HPP_
#define _ABS_FORMATION_CONTROLLER_HPP_

#include "formation_controller.hpp"
#include "../fixed_wing_lib/vector.hpp"
#include "../fixed_wing_lib/filter.hpp"
#include "../fixed_wing_lib/vertical_controller/tecs.hpp"

#define ABS_FORMATION_CONTROLLER_INFO(a) \
    cout << "[ABS_FORMATION_CONTROLLER_INFO]:" << a << endl

class ABS_FORMATION_CONTROLLER : public FORMATION_CONTROLLER
{
public:

   //***X:0.45 0.65 0.5 0.001 0.0008 Y:  0.008 0.5 0.7 0.005 0.0015这组搭配可以实现一定程度的编队loliter，编队效果不好
   //***有时候对小幅的变化不敏感？比如从直线刚切入到盘旋阶段，过渡期从机变化不大，需要误差到一定程度之后才会变化？分段？
    /* TECS控制器参数 */
    struct _s_tecs_params
    {
        int EAS2TAS{1};

        bool climboutdem{false};

        float climbout_pitch_min_rad{0.2};

        float speed_weight{1};

        float time_const_throt{8.0}; //8.0           //***调整TECS参数，看效果是否更好

        float time_const{5.0}; //5.0   //***试过调为1.0 没有影响
    };

    //***控制律参数
    struct _s_control_law_params
    {
        float k1{1.8};  //***与y_e(角速度)的收敛速度有关，越大速度越快   0.4 //***论文稳定性推导有问题，事实上k1小于2就可以
        float k2{30};  //***保证k1不太小，k2可以适当大一点
        float k3{1.0};                      //***与x_e(线速度)的收敛速度有关，越大速度越快   这里x指东西,正代表西，y南北，正为南(指QGC里的)
        float k4{0.2};     //*** k1 1.5效果比1.0好 1.8最大全局误差为3.0 1.5最大全局误差为3.8 1.9 在3.3左右
    };
    //***roll_2_pitch和height_2_speed参数
    struct _s_compensate_state_params
    {
        float RollToPitchPara_KP = 0.15;//0.1;
        float RollToPitchPara_OpenTh = 5;//10;
        float RollToPitchPara_Limit = 3;//2;
        float  HeiToSpeCtrlPara_ErrTh = 10;
        bool HeiToSpeCtrlPara_IsOpen = true;
        float HeiToSpeCtrlPara_KP  = 1; //1
        float HeiToSpeCtrlPara_Limit = 10;
        float HeiToSpeCtrlPara_PitchOpenTh = 5;
    };

    /*重置控制器*/
    void reset_formation_controller();

    /* 设定编队控制器参数（主管产生期望空速） */
    void set_mix_Xerr_params(struct _s_mix_Xerr_params &input_params);                             //****通过引用，定位到了task_main.cpp中设置的参数，而不是这里结构体定义的参数


    /* 设定TECS控制器参数 */
    void set_tecs_params(struct _s_tecs_params &input_params);

    //***导航律函数
    void control_law();         

    //***补偿滚转引起的掉高，试一下会不会起作用，还有究竟是什么原因导致飞机会降不到最低速度也有时候上升不到最大速度，TECS参数？去调一下油门哪个
    float roll_to_pitch(float aRoll);

    //***补偿高度方向的速度，将TECS的爬升门限设为10了，结果是飞机pitch一直为0附近，不过高度仍然会下降和上升
    float height_to_speed(float aHei, float cHei, float cPitch);                               

private:
    /**
   *编队控制器外函数，变量（组）
   */

    /* 绝对速度位置控制器时间戳 */
    long abs_pos_vel_ctrl_timestamp{0};

    /* 滤波后的领机信息 */
    _s_leader_states leader_states_f;

    /* 滤波后的从机信息 */
    _s_fw_states fw_states_f;

    /* 完成对于领机从机的滤波函数 */
    void filter_led_fol_states();

    /* 是否使用滤波器对原始数据滤波 */
    bool use_the_filter{true};
    
    /* 领机gol速度x滤波器 */
    FILTER led_gol_vel_x_filter;

    /* 领机gol速度y滤波器 */
    FILTER led_gol_vel_y_filter;

    /* 检验计算本机的空速（状态）以及实际读取的空速的合法性 */
    bool fw_airspd_states_valid{true};

    /* 检验计算领机的空速（状态）以及实际读取的空速的合法性 */
    bool led_airspd_states_valid{true};

    /* 领机空速向量 */
    Vec led_arispd;

    /* 领机地速向量 */
    Vec led_gspeed_2d;

    /* 本机空速向量 */
    Vec fw_arispd;

    /* 本机地速向量 */
    Vec fw_gspeed_2d;

    /* 本机风估计向量 */
    Vec fw_wind_vector;

    /* 领机dir_cos，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double led_cos_dir{0.0};

    /* 领机dir_sin，这其中的dir这个角度，可能是yaw，也可能是速度偏角*/
    double led_sin_dir{0.0};

    /* 本机dir_cos，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double fw_cos_dir{0.0};

    /* 本机dir_sin，这其中的dir这个角度，可能是yaw，也可能是速度偏角*/  //***与风速有关
    double fw_sin_dir{0.0};


    //***控制律
    _s_control_law_params control_law_params;
    //***补偿
    _s_compensate_state_params state_params;


    /* 重置内部控器标志量 */
    bool rest_speed_pid{false};

    /* 从机期望地速增量，最终实现的是领机与从机地速一致 */
    float del_fol_gspeed{0.0};

    /* 飞机期望空速（前一时刻） */
    float airspd_sp_prev{0.0};

    /* 飞机期望空速 */
    float airspd_sp{0.0};

    /* 本机空速期望值滤波器 */
    FILTER airspd_sp_filter;

    /**
   * TECS函数，变量（组）
   */

    /* TECS控制器 */
    TECS _tecs;

    /* 重置TECS */
    bool rest_tecs{false};

    /* 纵向速度有效标志位 */
    bool vz_valid{false};

    /* TECS参数 */
    _s_tecs_params tecs_params;


    /* 最终roll通道控制量 */
    float roll_cmd{0.0};

    /* 最终roll通道控制量 */
    float roll_cmd_prev{0.0};
    
    /* 期望滚转角滤波器 */
    FILTER roll_cmd_filter;

    /**
   * 其他计算函数，变量（组）
   */
    /* 原始信息预处理 */
    Point get_plane_to_sp_vector(Point origin, Point target);
};

#endif
