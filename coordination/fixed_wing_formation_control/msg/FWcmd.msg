#固定翼控制指令的期望值		
string cmd_mode

float64 pitch_angle_sp

float64 yaw_angle_sp

float64 roll_angle_sp

geometry_msgs/Quaternion att_sp_quater #姿态四元数，只能在外面弄下旋转矩阵了

float64 throttle_sp