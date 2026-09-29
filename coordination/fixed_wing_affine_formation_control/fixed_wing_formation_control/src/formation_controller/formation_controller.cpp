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
 * @Description: 设定编队期望队形
 */
void FORMATION_CONTROLLER::set_formation_type(int formation_type)
{
        if (planeID == 0)
        {
            formation_offset.xb = 0;          //***在全局误差系下，效果更好，主机机体系下要注意控制量产生的区别
            formation_offset.yb = 0;          
            formation_offset.zb = 0;            //***机体系下，考虑滚转角的限制，采用这种基于从机误差的方式，若期望编队点在主机转弯内侧，则从机需要更大的滚转角           
        }
        if (planeID == 1)
        {
            formation_offset.xb = -100;   
            formation_offset.yb = 100;
            formation_offset.zb = 0;
        }
        if (planeID == 2)
        {
            formation_offset.xb = -100;  
            formation_offset.yb = -100;
            formation_offset.zb = 0;
        }
        if (planeID == 3)
        {
            formation_offset.xb = -200;  
            formation_offset.yb = 200;
            formation_offset.zb = 0;
        }
        if (planeID == 4)
        {
            formation_offset.xb = -200;  
            formation_offset.yb = 0;
            formation_offset.zb = 0;
        }
        if (planeID == 5)
        {
            formation_offset.xb = -200;  
            formation_offset.yb = -200;
            formation_offset.zb = 0;
        }

}
/**
 * @Input: void
 * @Output: void
 * @Description: 更新长、僚机状态飞行状态
 */
void FORMATION_CONTROLLER::update_everyfw_states(const struct _s_fw_states *everyfw_states0,
                                                 const struct _s_fw_states *everyfw_states1,
                                                 const struct _s_fw_states *everyfw_states2,
                                                 const struct _s_fw_states *everyfw_states3,
                                                 const struct _s_fw_states *everyfw_states4,
                                                 const struct _s_fw_states *everyfw_states5,
                                                 const struct _s_fw_states *thisfw_states)
{ /* 使用指针，避免内存浪费 */
    
    everyfwstates0 = *everyfw_states0;
    everyfwstates1 = *everyfw_states1;  
    everyfwstates2 = *everyfw_states2;     
    everyfwstates3 = *everyfw_states3;  
    everyfwstates4 = *everyfw_states4;  
    everyfwstates5 = *everyfw_states5;  
    fw_states = *thisfw_states;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 更新长和长机邻居状态飞行状态
 */
void FORMATION_CONTROLLER::update_leaders_states(float current_time,
                                                 const float *itsneighborstates0,
                                                 const float *itsneighborstates1,
                                                 const struct _s_fw_states *thisfw_states)
{   currenttime=current_time;
    /* 使用指针，避免内存浪费 */
    itsneighbor_scurve0 = *itsneighborstates0;
    itsneighbor_scurve1 = *itsneighborstates1;
    fw_states = *thisfw_states;
    FORMATION_CONTROLLER_INFO("更新航向角"<<fw_states.yaw_angle);
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
        return 0;
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
