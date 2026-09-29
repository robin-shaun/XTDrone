/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */
#ifndef _ABS_FORMATION_CONTROLLER_HPP_ //防止多次包含同一个文件
#define _ABS_FORMATION_CONTROLLER_HPP_

#include "formation_controller.hpp"
#include "../fixed_wing_lib/vector.hpp"
#include "../fixed_wing_lib/filter.hpp"
#include "../fixed_wing_lib/vertical_controller/tecs.hpp"
#include "../fixed_wing_lib/syslib.hpp"


#define ABS_FORMATION_CONTROLLER_INFO(a) \
    cout << "[ABS_FORMATION_CONTROLLER_INFO]:" << a << endl

class ABS_FORMATION_CONTROLLER : public FORMATION_CONTROLLER
{
public:

   //***X:0.45 0.65 0.5 0.001 0.0008 Y:  0.008 0.5 0.7 0.005 0.0015这组搭配可以实现一定程度的编队loliter，编队效果不好
   //***有时候对小幅的变化不敏感？比如从直线刚切入到盘旋阶段，过渡期从机变化不大，需要误差到一定程度之后才会变化？分段？
    int planeID{0};
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
        /*无人机数量*/
        int NumLeader{3};   /*leader总数*/
        int NumFollower{3}; /*follower总数*/
        int NumUAV{6};      /*无人机总数*/
        /* 单长机路径跟随控制律参数 */
        float kd{0.01};    /*kd0.01*/
        float komega{2};  /*komega*/
        float ks{1};      /*dotL*/       
        float kpi{0.2*PI};  /*kpi*/
        /* 多长机协同控制律参数 */
        float gammad{10};   /*0.12*/
        float beta{2};      /*0.01*/
        float ktheta{5};    /*5*/
        /* 僚机仿射编队控制律参数 */
        float Omega[6][6]={                                 /*定义等边三角形应力矩阵omega*/
            {0.2887,-0.2887,-0.2887,0,0.2887,0},
            {-0.2887,0.8660,-0.2887,-0.2887,-0.2887,0.2887},
            {-0.2887,-0.2887,0.8660,0.2887,-0.2887,-0.2887},
            {0,-0.2887,0.2887,0.2887,-0.2887,0},
            {0.2887,-0.2887,-0.2887,-0.2887,0.8660,-0.2887},
            {0,0.2887,-0.2887,0,-0.2887,0.2887}
        };
        float K[6][6]={
            {-1.0000,0,2.0000,0,0.0000,0},
            {0,-1.0000,0,2.0000,0,0.0000},
            {-1.0000,0,1.0000,0,1.0000,0},
            {0,-1.0000,0,1.0000,0,1.0000},
            {-1.0000,0,2.0000,0,0.0000,0},
            {0,-1.0000,0,0.0000,0,2.0000}
        };
       

    };

    //障碍物位置
    struct _s_control_obstacle_avoidance
    {
        float obstacle_num=5;
        float obstacle_pos[2][5]={
            {1000,2000,1500,1600,1200},//x
            {1000,1800,1200,1500,1300}//y
        };
        float obstacle_radius=50;//障碍物半径
        float uav_radius=2;//无人机半径
        float staticEdgeDistance=10;//静态安全距离
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

    //***长机导航律函数
    void control_law();      

    //***僚机偏移控制律
    void follower_control_law();
    //**避障算法 */
    void obstacle_avoidance(float pos_x,float pos_y);
    //***将角度归到[-pi,pi)的区间内
    float RoundAngle(float ang);  

    void set_ID(int id); 

private:
    
    float nowtime{0};
    /* 控制时间间隔 */
    float _dt{0.01};
    /* 控制时间间隔max */
    float _dtMax{0.1};
    /* 控制时间间隔min */
    float _dMin{0.01};
    /*避障输出命令*/
    float vel_cmd{0.0f};
    float angular_rate_cmd{0.0f};
    float is_avoid=false;//避障标志
    
    //***编队控制器外函数，变量（组）

    /* 绝对速度位置控制器时间戳 */
    long abs_pos_vel_ctrl_timestamp{0};

    /* 滤波后的长机邻居信息，计算后的 */
    float its_neighbor_scurve[2]={0.0,0.0};

    /* 滤波后的本机信息，计算后的 */
    _s_fw_states fw_states_f;

    /* 滤波后的所有无人机信息，计算后的 */
    _s_fw_states everyfw_states_f0;
    _s_fw_states everyfw_states_f1;
    _s_fw_states everyfw_states_f2;
    _s_fw_states everyfw_states_f3;
    _s_fw_states everyfw_states_f4;
    _s_fw_states everyfw_states_f5;
    

    /* 僚机控制律：完成对于长机和僚机的滤波函数 */
    void filter_fw_states();


    /* 是否使用滤波器对原始数据滤波 */
    bool use_the_filter{true};
    
    /* 领机gol速度x滤波器 */
    FILTER led_gol_vel_x_filter;

    /* 领机gol速度y滤波器 */
    FILTER led_gol_vel_y_filter;

    /* 检验计算长机1的空速（状态）以及实际读取的空速的合法性 */
    bool fw_airspd_states_valid{true};


    /* 检验计算领机的空速（状态）以及实际读取的空速的合法性 */
    bool led_airspd_states_valid{true};

    /* 检验计算僚机的长机的空速（状态）以及实际读取的空速的合法性 */
    bool itsled_airspd_states_valid{true};

    /* 僚机控制律->僚机空速向量 */
    Vec fw_arispd;

    /* 僚机控制律->僚机地速向量 */
    Vec fw_gspeed_2d; 

    /* 僚机控制律->长机空速向量 */
    Vec lead_arispd0;
    Vec lead_arispd1;
    Vec lead_arispd2;

    /* 僚机控制律->长地速向量 */
    Vec lead_gspeed_2d0;  
    Vec lead_gspeed_2d1;  
    Vec lead_gspeed_2d2;

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

    _s_control_obstacle_avoidance obstacle_params;
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

    /* 重置TECS标志位 */
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
