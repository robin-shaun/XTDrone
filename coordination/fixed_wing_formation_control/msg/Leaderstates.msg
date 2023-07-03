#本消息是定义的领机的状态消息，同样也包含了相对运动的关系

#如果能得到领机的当前姿态信息，相当于协同
float64 pitch_angle

float64 yaw_angle

bool yaw_valid

float64 roll_angle

float64 thrust#这个有点不好获得

#领机的绝对位置速度信息
float64 latitude

float64 altitude

float64 longitude

float64 relative_alt

float64 airspeed

float64 ned_vel_x

float64 ned_vel_y

float64 ned_vel_z

float64 global_vel_x

float64 global_vel_y

float64 global_vel_z

float64 ned_acc_x

float64 ned_acc_y

float64 ned_acc_z

float64 body_acc_x

float64 body_acc_y

float64 body_acc_z

#领机角速度信息
float64 yaw_rate

float64 roll_rate

float64 pitch_rate

#领机与本机相对位置速度信息
float64 rel_pos_Xb

float64 rel_pos_Yb

float64 rel_pos_Zb

float64 rel_vel_Xb

float64 rel_vel_Yb

float64 rel_vel_Zb

float64 rel_angle_a#相对角度，之后再改名

float64 rel_angle_b

float64 rel_angle_c

float64 rel_att_pitch

float64 rel_att_roll

float64 rel_att_yaw