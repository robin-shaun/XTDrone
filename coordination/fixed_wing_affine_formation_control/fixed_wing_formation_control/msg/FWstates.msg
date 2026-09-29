# Message 固定翼的编队控制，可能用到的消息类型全定义

uint16 planeID #无人机总编号1234...

float64 scurve #虚拟目标点弧长信息

uint16 gpID #无人长机编号，对应同一架长机的编号相同[1,1,...1;2,2,...,2;......10,10,...,10]

uint16 idIngp #无人机组内编号[1,2,3,4,5,6,7,8,9,10]

uint16 patnID #队形的id,只有长机到了一特定的航点时才更新

uint16 posID #无人机在队形中的位置序号[1,2,3,4,...,10,1,2,3,...10,..,1,2,3,..,10]

bool altitude_lock

bool in_air

string control_mode

float64 pitch_angle

float64 yaw_angle

bool yaw_valid

float64 roll_angle

geometry_msgs/Quaternion att_quater #姿态四元数，只能在外面弄下旋转矩阵了

float64 ground_speed_ned_n

float64 ground_speed_ned_e

float64 ground_speed_ned_d

float64 ground_speed

float64 global_vel_x

float64 global_vel_y

float64 global_vel_z

float64 air_speed

float64 wind_estimate_x

float64 wind_estimate_y

float64 wind_estimate_z

float64 relative_hight

float64 latitude

float64 altitude

float64 longitude

float64 relative_alt

float64 ned_altitude

float64 ned_pos_x

float64 ned_pos_y

float64 ned_pos_z

float64 ned_vel_x

float64 ned_vel_y

float64 ned_vel_z

float64 ned_acc_x

float64 ned_acc_y

float64 ned_acc_z

geometry_msgs/Vector3 ned_acc

float64 body_acc_x

float64 body_acc_y

float64 body_acc_z

geometry_msgs/Vector3 body_acc

float64 battery_voltage

float64 battery_precentage

float64 battery_current

#角速度
float64 yaw_rate

float64 pitch_rate

float64 roll_rate
