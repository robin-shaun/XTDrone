import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String


MAX_LIN_VEL = 10
MAX_ANG_VEL = 0.1
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01

cmd_vel_mask = False
ctrl_leader = False

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
s   : hover(offboard mode) and remove the mask of keyboard control
k   : hover(hover mode) and remove the mask of keyboard control
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
s or k : hover and remove the mask of keyboard control
0~9 : extendable mission(eg.different formation configuration)
      this will mask the keyboard control
g   : control all drones
CTRL-C to quit
"""

e = """
Communications Failed
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
        
def vels(target_forward_vel, target_leftward_vel, target_upward_vel,target_angular_vel):
    return "currently:\t forward x %s\t leftward y %s\t upward z %s\t angular %s " % (target_forward_vel, target_leftward_vel, target_upward_vel, target_angular_vel)


if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    control_type = sys.argv[1]
    
    cmd= String()
    twist = Twist()   
    
    rospy.init_node('multirotor_keyboard_control')
    
    if control_type == 'vel':
        cmd_vel_flu_pub = rospy.Publisher('/xtdrone/cmd_vel_flu', Twist, queue_size=10)
        cmd_pub = rospy.Publisher('/xtdrone/cmd',String,queue_size=10)
        leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=10)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader_cmd", String, queue_size=10)
 
    else:
        cmd_accel_flu_pub[i] = rospy.Publisher('/xtdrone/cmd_accel_flu', Twist, queue_size=10)
        cmd_pub[i] = rospy.Publisher('/xtdrone/cmd',String,queue_size=10)
        leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=10)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader_cmd", String, queue_size=10)

    forward  = 0.0
    leftward  = 0.0
    upward  = 0.0
    angular = 0.0

    print_msg()
    while(1):
        key = getKey()
        if key == 'w' :
            forward = forward + LIN_VEL_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'x' :
            forward = forward - LIN_VEL_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'a' :

            leftward = leftward + LIN_VEL_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'd' :
            leftward = leftward - LIN_VEL_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == 'i' :
            upward = upward + LIN_VEL_STEP_SIZE
            print_msg()
            if control_type == 'vel':
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            else:
                print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))

        elif key == ',' :
            upward = upward - LIN_VEL_STEP_SIZE
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
            #cmd = 'AUTO.TAKEOFF'
            cmd = ''
            print(msg)
            print('Takeoff mode is disenabled now')
        elif key == 'b':
            cmd = 'OFFBOARD'
            print_msg()
            print('Offboard')
        elif key == 'n':
            cmd = 'AUTO.LAND'
            print_msg()
            print('Landing')
        elif key == 'g':
            ctrl_leader = not ctrl_leader
            print_msg()
        elif key == 'k':
            cmd = 'HOVER'
            cmd_vel_mask = False
            print_msg()
            print('Hover')
        elif key == 's' :
            cmd_vel_mask = False
            forward   = 0.0
            leftward   = 0.0
            upward   = 0.0
            angular  = 0.0
            forward  = 0.0
            leftward  = 0.0
            upward  = 0.0
            angular = 0.0
            cmd = 'HOVER'
            print_msg()
            print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (forward, leftward, upward, angular))
            print('Hover')
        else:
            for i in range(10):
                if key == str(i):
                    cmd = 'mission'+key
                    print_msg()
                    print(cmd)
                    cmd_vel_mask = True
            if (key == '\x03'):
                break

        if forward > MAX_LIN_VEL:
            forward = MAX_LIN_VEL
        elif forward < -MAX_LIN_VEL:
            forward = -MAX_LIN_VEL
        if leftward > MAX_LIN_VEL:
            leftward = MAX_LIN_VEL
        elif leftward < -MAX_LIN_VEL:
            leftward = -MAX_LIN_VEL
        if upward > MAX_LIN_VEL:
            upward = MAX_LIN_VEL
        elif upward < -MAX_LIN_VEL:
            upward = -MAX_LIN_VEL
            
        twist.linear.x = -leftward; twist.linear.y = forward ; twist.linear.z = upward
        twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular

        if ctrl_leader:
            if control_type == 'vel':
                leader_cmd_vel_flu_pub.publish(twist)
            else:
                leader_cmd_aceel_flu_pub.publish(twist)
            leader_cmd_pub.publish(cmd)
        else:
            if not cmd_vel_mask:
                if control_type == 'vel':
                    cmd_vel_flu_pub.publish(twist)   
                else:
                    cmd_accel_flu_pub.publish(twist)
            cmd_pub.publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
