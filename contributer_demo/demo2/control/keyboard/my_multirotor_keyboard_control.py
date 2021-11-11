# encoding: utf-8
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String


MAX_LINEAR = 20
MAX_ANG_VEL = 3
LINEAR_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

cmd_vel_mask = False
ctrl_leader = False #初始 控制所有无人机

msg2all = """
Control Your XTDrone!
To all drones  (press g to control the leader)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward velocity 
a/d : increase/decrease leftward velocity
i/, : increase/decrease upward velocity
j/l : increase/decrease orientation
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover and remove the mask of keyboard control
0~9 : extendable mission(eg.different formation configuration)
      this will mask the keyboard control
g   : control the leader
CTRL-C to quit
"""

msg2leader = """
Control Your XTDrone!
To the leader  (press g to control all drones)
---------------------------
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward velocity
a/d : increase/decrease leftward velocity
i/, : increase/decrease upward velocity
j/l : increase/decrease orientation
r   : return home
t/y : arm/disarm
v/n : takeoff(disenabled now)/land
b   : offboard
s/k : hover and remove the mask of keyboard control
0~9 : extendable mission(eg.different formation configuration)
      this will mask the keyboard control
g   : control all drones
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey(): # 获得键盘按键
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_msg(): # 打印提示信息
    if ctrl_leader:
        print(msg2leader)
    else:
        print(msg2all)      

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    multirotor_type = sys.argv[1]
    multirotor_num = int(sys.argv[2])
    control_type = sys.argv[3]  #vel 速度控制
    '''
    if multirotor_num == 18: #多无人机编队
        formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']  # 0-9的参数
    elif multirotor_num == 9:
        formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
    elif multirotor_num == 6:
        formation_configs = ['waiting', 'T', 'diamond', 'triangle']
    '''          #               0       1       2           3
    if multirotor_num == 21:
        formation_configs = ['waiting', 'cube21', 'circle21', 'pyramid21', 'diamond21']
    elif multirotor_num == 34:
        formation_configs = ['waiting', 'CUBE', 'HEART', '520', 'NUDT', '八一']

    cmd= String()
    twist = Twist()   
    
    rospy.init_node('multirotor_keyboard_multi_control')
    
    if control_type == 'vel':  # 速度控制
        multi_cmd_vel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_vel_flu', Twist, queue_size=1)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=1)
        leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)
 
    else:  # 加速度控制
        multi_cmd_accel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_accel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_accel_flu', Twist, queue_size=1)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=1)
        leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=1)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)

    forward  = 0.0
    leftward  = 0.0
    upward  = 0.0
    angular = 0.0

    print_msg()
    while(1):
        key = getKey()
        if key == 'w' :
            forward = forward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'x' :
            forward = forward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'a' :

            leftward = leftward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'd' :
            leftward = leftward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'i' :
            upward = upward + LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == ',' :
            upward = upward - LINEAR_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'j':
            angular = angular + ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'l':
            angular = angular - ANG_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'r':  # 自动返回起飞点
            cmd = 'AUTO.RTL'
            print_msg()
            print('Returning home')
        elif key == 't':  #飞机解锁
            cmd = 'ARM'
            print_msg()
            print('Arming')
        elif key == 'y':  # 飞机上锁
            cmd = 'DISARM'
            print_msg()
            print('Disarming')
        elif key == 'v':
            cmd = 'AUTO.TAKEOFF'
            cmd = ''
            print_msg()
            #print('Takeoff mode is disenabled now')
        elif key == 'b':
            cmd = 'OFFBOARD'
            print_msg()
            print('Offboard')
        elif key == 'n':  # 自动着陆
            cmd = 'AUTO.LAND'
            print_msg()
            print('Landing')
        elif key == 'g':   # 开启或结束编队控制
            ctrl_leader = not ctrl_leader
            print_msg()
        elif key in ['k', 's']:  # 悬停，全部参数归零
            cmd_vel_mask = False
            forward   = 0.0
            leftward   = 0.0
            upward   = 0.0
            angular  = 0.0
            cmd = 'HOVER'
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            print('Hover')
        else:
            for i in range(10): # 按键数字
                if key == str(i):
                    cmd = formation_configs[i]  # 编队的队形
                    print_msg()
                    print(cmd)
                    cmd_vel_mask = True  # 有掩模，其他按键无效
            if (key == '\x03'):
                break

        if forward > MAX_LINEAR:
            forward = MAX_LINEAR
        elif forward < -MAX_LINEAR:
            forward = -MAX_LINEAR
        if leftward > MAX_LINEAR:
            leftward = MAX_LINEAR
        elif leftward < -MAX_LINEAR:
            leftward = -MAX_LINEAR
        if upward > MAX_LINEAR:
            upward = MAX_LINEAR
        elif upward < -MAX_LINEAR:
            upward = -MAX_LINEAR
        if angular > MAX_ANG_VEL:
            angular = MAX_ANG_VEL
        elif angular < -MAX_ANG_VEL:
            angular = - MAX_ANG_VEL
            
        twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
        twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular

        for i in range(multirotor_num): # 无人机数量
            if ctrl_leader:
                if control_type == 'vel':
                    leader_cmd_vel_flu_pub.publish(twist)
                else:
                    leader_cmd_aceel_flu_pub.publish(twist)  #无定义
                leader_cmd_pub.publish(cmd)
            else:
                if not cmd_vel_mask: #只有在输入0-9时，mask为True，其余情况为F 。这里是非数字时
                    if control_type == 'vel':
                        multi_cmd_vel_flu_pub[i].publish(twist)   
                    else:
                        multi_cmd_accel_flu_pub[i].publish(twist)
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
