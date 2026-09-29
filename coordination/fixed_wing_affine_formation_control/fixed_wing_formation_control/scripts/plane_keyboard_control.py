import rospy
from geometry_msgs.msg import Pose
import sys, select, os
import tty, termios
from std_msgs.msg import String
from fixed_wing_formation_control.msg import FWcmd


LIN_STEP_SIZE = 0.1
ANG_STEP_SIZE = 0.01


ctrl_leader = False
send_flag = False

msg2all = """
Control Your XTDrone!
To all drones  (press g to control the leader)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease north setpoint  
a/d : increase/decrease east setpoint 
i/, : increase/decrease upward setpoint
j/l : increase/decrease orientation
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s   : loiter
k   : idle
0~9 : extendable mission(eg.different formation configuration)
      this will mask the keyboard control
g   : control the leader
o   : send setpoint
CTRL-C to quit
"""

msg2leader = """
Control Your XTDrone!
To the leader (press g to control all drones)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease north setpoint  
a/d : increase/decrease east setpoint 
i/, : increase/decrease upward setpoint
j/l : increase/decrease orientation
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s   : loiter
k   : idle
0~9 : extendable mission(eg.different formation configuration)
g   : control all drones
o   : send setpoint
CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_msg():
    if ctrl_leader:
        print(msg2leader)
    else:
        print(msg2all)

def ros_sub_and_pub():  # 【订阅】【自定义】订阅长机来自上层控制器的四通道控制量,固定翼控制指令的期望值
    fixed_wing_cmd_from_controller_sub = rospy.Subscriber(str(uavID)+"fixed_wing_formation_control/fw_cmd", FWcmd,  queue_size=10)

def pack_fw_states():
    fixed_wing_sub_pub.fw_states_form_mavros.control_mode =fixed_wing_sub_pub.current_state.mode

    fixed_wing_sub_pub.fw_states_form_mavros.scurve = fixed_wing_sub_pub.cmd_from_controller.scurve
# 以下为GPS信息
    fixed_wing_sub_pub.fw_states_form_mavros.altitude =fixed_wing_sub_pub.global_position_form_px4.altitude      //GPS高度获取不合适
    fixed_wing_sub_pub.fw_states_form_mavros.latitude =fixed_wing_sub_pub.global_position_form_px4.latitude
    fixed_wing_sub_pub.fw_states_form_mavros.longitude =fixed_wing_sub_pub.global_position_form_px4.longitude

# GPS速度是在ned下的，
    fixed_wing_sub_pub.fw_states_form_mavros.global_vel_x =fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.y
    fixed_wing_sub_pub.fw_states_form_mavros.global_vel_y =fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.x
    fixed_wing_sub_pub.fw_states_form_mavros.global_vel_z =-fixed_wing_sub_pub.velocity_global_fused_from_px4.twist.linear.z

    fixed_wing_sub_pub.fw_states_form_mavros.relative_alt =fixed_wing_sub_pub.global_rel_alt_from_px4.data

#以下为机体系和地面系的夹角，姿态角

    fixed_wing_sub_pub.fw_states_form_mavros.roll_angle =fixed_wing_sub_pub.att_angle_Euler[0]
    fixed_wing_sub_pub.fw_states_form_mavros.pitch_angle =-fixed_wing_sub_pub.att_angle_Euler[1] #添加负号转换到px4的系                           姿态角是从IMU中获得四元数，然后转化为欧拉角
                                                                                                                                                   # 应该和local_position/pose一致

    if -fixed_wing_sub_pub.att_angle_Euler[2] + deg_2_rad(90.0) > 0:
        fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle =-fixed_wing_sub_pub.att_angle_Euler[2] +deg_2_rad(90.0); 
    #添加符号使增加方向相同，而且领先于px490°                    //****这里为什么让yaw领先90度未知
    else:
        fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle =-fixed_wing_sub_pub.att_angle_Euler[2] + deg_2_rad(90.0) +deg_2_rad(360.0);

# 姿态四元数赋值
    att_angle[0] = fixed_wing_sub_pub.fw_states_form_mavros.roll_angle
    att_angle[1] = fixed_wing_sub_pub.fw_states_form_mavros.pitch_angle
    att_angle[2] = fixed_wing_sub_pub.fw_states_form_mavros.yaw_angle
    euler_2_quaternion(att_angle, att_quat)
    fixed_wing_sub_pub.fw_states_form_mavros.att_quater.w = att_quat[0]
    fixed_wing_sub_pub.fw_states_form_mavros.att_quater.x = att_quat[1]
    fixed_wing_sub_pub.fw_states_form_mavros.att_quater.y = att_quat[2]
    fixed_wing_sub_pub.fw_states_form_mavros.att_quater.z = att_quat[3]

# 以下为ned坐标系下的位置，速度
    fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_x =fixed_wing_sub_pub.local_position_from_px4.pose.position.y
    fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_y =fixed_wing_sub_pub.local_position_from_px4.pose.position.x
    fixed_wing_sub_pub.fw_states_form_mavros.ned_pos_z =-fixed_wing_sub_pub.local_position_from_px4.pose.position.z

    fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_x =fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.y
    fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_y =fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.x
    fixed_wing_sub_pub.fw_states_form_mavros.ned_vel_z =-fixed_wing_sub_pub.velocity_ned_fused_from_px4.twist.linear.z

# 以下为体轴系加速度，体轴系当中的加速度是符合px4机体系的定义的
    fixed_wing_sub_pub.fw_states_form_mavros.body_acc_x =fixed_wing_sub_pub.imu.linear_acceleration.x
    fixed_wing_sub_pub.fw_states_form_mavros.body_acc_y =fixed_wing_sub_pub.imu.linear_acceleration.y
    fixed_wing_sub_pub.fw_states_form_mavros.body_acc_z =fixed_wing_sub_pub.imu.linear_acceleration.z

    fixed_wing_sub_pub.fw_states_form_mavros.body_acc.x =fixed_wing_sub_pub.imu.linear_acceleration.x
    fixed_wing_sub_pub.fw_states_form_mavros.body_acc.y =fixed_wing_sub_pub.imu.linear_acceleration.y
    fixed_wing_sub_pub.fw_states_form_mavros.body_acc.z =fixed_wing_sub_pub.imu.linear_acceleration.z

  #以下来自altitude
    fixed_wing_sub_pub.fw_states_form_mavros.relative_hight =fixed_wing_sub_pub.altitude_from_px4.relative
    fixed_wing_sub_pub.fw_states_form_mavros.ned_altitude =fixed_wing_sub_pub.altitude_from_px4.local

  #空速和地速
    fixed_wing_sub_pub.fw_states_form_mavros.air_speed =fixed_wing_sub_pub.air_ground_speed_from_px4.airspeed
    fixed_wing_sub_pub.fw_states_form_mavros.ground_speed =fixed_wing_sub_pub.air_ground_speed_from_px4.groundspeed

  #风估计
    fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_x =fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.y
    fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_y =fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.x
    fixed_wing_sub_pub.fw_states_form_mavros.wind_estimate_z =-fixed_wing_sub_pub.wind_estimate_from_px4.twist.twist.linear.z
  #电池状态
    fixed_wing_sub_pub.fw_states_form_mavros.battery_current =fixed_wing_sub_pub.battrey_state_from_px4.current
    fixed_wing_sub_pub.fw_states_form_mavros.battery_precentage =fixed_wing_sub_pub.battrey_state_from_px4.percentage
    fixed_wing_sub_pub.fw_states_form_mavros.battery_voltage =fixed_wing_sub_pub.battrey_state_from_px4.voltage

# 角速度
# 参照角度赋值部分的正负号添加形式 感觉没问题 看看发布的话题FWstates的正负
    fixed_wing_sub_pub.fw_states_form_mavros.pitch_rate = -fixed_wing_sub_pub.imu.angular_velocity.y
    fixed_wing_sub_pub.fw_states_form_mavros.roll_rate = fixed_wing_sub_pub.imu.angular_velocity.x
    fixed_wing_sub_pub.fw_states_form_mavros.yaw_rate = -fixed_wing_sub_pub.imu.angular_velocity.z

# 把订阅的所有消息组成包，发布出去；打包的mavros发布
    fixed_wing_states_pub.publish(fixed_wing_sub_pub.fw_states_form_mavros)



if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    plane_num = int(sys.argv[1])
    rospy.init_node('plane_keyboard_control')
    multi_cmd_pose_enu_pub = [None]*plane_num
    multi_cmd_pub = [None]*plane_num
    for i in range(plane_num):
        multi_cmd_pose_enu_pub[i] = rospy.Publisher('/uav_'+str(i)+'/cmd_pose_enu', Pose, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/uav_'+str(i)+'/cmd',String,queue_size=1)
    leader_pose_pub = rospy.Publisher("/uav/leader/cmd_pose", Pose, queue_size=1)
    leader_cmd_pub = rospy.Publisher("/uav/leader_cmd", String, queue_size=3)
    cmd= String()
    pose = Pose()    

    forward  = 0.0
    leftward  = 0.0
    upward  = 0.0
    angular = 0.0

    print_msg()
    while(1):
        key = getKey()
        if key == 'w' :
            forward = forward + LIN_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == 'x' :
            forward = forward - LIN_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == 'a' :
            leftward = leftward + LIN_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == 'd' :
            leftward = leftward - LIN_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == 'i' :
            upward = upward + LIN_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == ',' :
            upward = upward - LIN_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == 'j':
            angular = angular + ANG_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == 'l':
            angular = angular - ANG_STEP_SIZE
            print_msg()
            print("currently:\t north %.2f\t east %.2f\t upward %.2f\t angular %.2f " % (forward, leftward, upward, angular))
        elif key == 'r':
            cmd = 'AUTO.RTL'
            print_msg()
            print('Returning home')
        elif key == 't':
            cmd = 'ARM'
            print_msg()
            print('Arming')
        elif key == 'y':
            cmd = 'DISARM'
            print_msg()
            print('Disarming')
        elif key == 'v':
            cmd = 'AUTO.TAKEOFF'
            print_msg()
            print('AUTO.TAKEOFF')
        elif key == 'b':
            cmd = 'OFFBOARD'
            print_msg()
            print('Offboard')
        elif key == 'n':
            cmd = 'AUTO.LAND'
            print_msg()
            print('AUTO.LAND')
        elif key == 'g':
            ctrl_leader = not ctrl_leader
            print_msg()
        elif key == 's':
            cmd = 'loiter'
            print_msg()
            print('loiter')
        elif key == 'k' :
            cmd = 'idle'
            print_msg()
            print('idle')
        elif key == 'o':
            send_flag = True
            print_msg()
            print('send setpoint')
        elif key == str(1):
            cmd = 'OFFBOARD'
            print_msg()
            ros_sub_and_pub()
            print('Formation control start!')
        else:
            if (key == '\x03'):
                break
            

        pose.position.x = forward; pose.position.y = leftward; pose.position.z = upward
        
        pose.orientation.x = 0.0; pose.orientation.y = 0.0;  pose.orientation.z = angular
           
        for i in range(plane_num):
            if ctrl_leader:
                if send_flag:
                    leader_pose_pub.publish(pose)
                leader_cmd_pub.publish(cmd)
            else:
                if send_flag:
                    multi_cmd_pose_enu_pub[i].publish(pose)    
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
