#include "uav_track_car/pid_interface.hpp"

PIDInterface::PIDInterface(float k_p, float k_i, float k_d) : k_p_(k_p), k_i_(k_i), k_d_(k_d){

}

PIDInterface::~PIDInterface(){

}

float PIDInterface::Control(float target_value, float actual_value){
    float E_k = target_value - actual_value;

    // 比例控制
    float output_p = E_k * k_p_;

    // 积分控制
    history_integral = history_integral + E_k;
    float output_i = history_integral * k_i_;

    // 微分控制
    float output_d = (E_k - history_E ) * k_d_;
    history_E = E_k;
    return output_p + output_d + output_i;

}