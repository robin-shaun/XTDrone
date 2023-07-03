/*
 * @------------------------------------------1: 1------------------------------------------@
 * @修改自  lee-shun Email: 2015097272@qq.com
 */

#include "formation_controller.hpp"

void FORMATION_CONTROLLER::set_fw_planeID(int id)
{
    planeID = id;
}
/**
 * @Input: void
 * @Output: void
 * @Description: 更新飞机，领机飞行状态
 */
void FORMATION_CONTROLLER::update_led_fol_states(const struct _s_leader_states *leaderstates,
                                                 const struct _s_fw_states *thisfw_states)
{ /* 使用指针，避免内存浪费 */
    leader_states = *leaderstates;
    fw_states = *thisfw_states;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 设定编队期望队形
 */
void FORMATION_CONTROLLER::set_formation_type(int formation_type)
{
    switch (formation_type)
    {
    case 1:     //***南北一字编队
        if (planeID == 1)
        {
            formation_offset.xb = 30;
            formation_offset.yb = 0;
            formation_offset.zb = 0;  
        }
        else  //***从机2，由于只有从机执行编队控制，所以不需要主机的编号
        {
            formation_offset.xb = -30;
            formation_offset.yb = 0;
            formation_offset.zb = 0;
        }       
        break;
    
    //***接下来设置三架无人机的情况
    case 2:
        if (planeID == 1)
        {
            formation_offset.xb = 30;          //***在全局误差系下，效果更好，主机机体系下要注意控制量产生的区别
            formation_offset.yb = -30;          
            formation_offset.zb = 0;            //***机体系下，考虑滚转角的限制，采用这种基于从机误差的方式，若期望编队点在主机转弯内侧，则从机需要更大的滚转角           
        }
        else
        {
            formation_offset.xb = -30;   //***从机2  这样应该是三角形编队
            formation_offset.yb = -30;
            formation_offset.zb = 0;
        }
        break;

    case 3:
        if (planeID == 1)  //东西一字
        {
            formation_offset.xb = 0;
            formation_offset.yb = 30;
            formation_offset.zb = 0; 
        }
        else
        {
            formation_offset.xb = 0;
            formation_offset.yb = -30;
            formation_offset.zb = 0;
        }
        break;
    }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 设定编队控制器内部飞机模型函数，例如最大滚转角速度等
 */
//***这个方法后续并没有用上，实际的飞机参数直接通过结构体赋值了，在此作为一个备选功能，方便以后外部输入
//***去掉此方法不影响最后结果
void FORMATION_CONTROLLER::set_fw_model_params(struct _s_fw_model_params &input_params)
{
    fw_params = input_params;
}

/**
 * @Input: void
 * @Output: bool
 * @Description: 判断飞机传入的状态值是否有问题，是否在飞行之中
 */

/*这里作者原始代码有误，根据mavros/globoal/position/raw/gps_vel可知速度是有负值的，应取绝对值*/
bool FORMATION_CONTROLLER::identify_led_fol_states()
{
    if ((abs(leader_states.global_vel_x) > 3.0) ||
        (abs(leader_states.global_vel_y) > 3.0) ||
        (leader_states.relative_alt > 3.0))
    {
        led_in_fly = true;
    }
    else
    {
        led_in_fly = false;

        FORMATION_CONTROLLER_INFO("警告：领机未在飞行之中");
    }

    if ((abs(fw_states.global_vel_x) > 3.0) ||
        (abs(fw_states.global_vel_y) > 3.0) ||
        (fw_states.relative_alt > 3.0))
    {
        fol_in_fly = true;
    }
    else
    {
        fol_in_fly = false;

        FORMATION_CONTROLLER_INFO("警告：本机未在飞行之中");
    }

    if (led_in_fly && fol_in_fly)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @Input: void
 * @Output: void
 * @Description: 获得飞机期望的四通道控制量
 */
void FORMATION_CONTROLLER::get_formation_4cmd(struct _s_4cmd &fw_cmd)
{
    fw_cmd = _cmd;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 得到编队中本机的运动学期望值
 */
void FORMATION_CONTROLLER::get_formation_sp(struct _s_fw_sp &formation_sp)
{
    formation_sp = fw_sp;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 得到编队控制误差
 */
void FORMATION_CONTROLLER::get_formation_error(struct _s_fw_error &formation_error)
{
    formation_error = fw_error;
}
