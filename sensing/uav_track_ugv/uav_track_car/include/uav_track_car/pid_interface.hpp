#ifndef MAXROBOT_CONTROL_MODELS_PID_INTERFACE_HPP
#define MAXROBOT_CONTROL_MODELS_PID_INTERFACE_HPP

#include <iostream>

class PIDInterface{
public:
    PIDInterface(float k_p, float k_i, float k_d);
    ~PIDInterface();

public:
    float Control(float target_value, float actual_value);

private:
    float k_p_;           //比例系数
    float k_i_;
    float k_d_;

    float history_integral = 0.0;
    float history_E  = 0.0;
};


#endif