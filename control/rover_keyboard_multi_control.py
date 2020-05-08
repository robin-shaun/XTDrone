import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String


MAX_LIN_VEL = 20
MAX_ANG_VEL = 0.1
LIN_VEL_STEP_SIZE = 0.1
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

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input
    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    rover_num = int(sys.argv[1])
    rospy.init_node('rover_keyboard_multi_control')
    multi_cmd_vel_flu_pub = [None]*rover_num
    multi_cmd_pub = [None]*rover_num
    for i in range(rover_num):
        multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/rover_'+str(i)+'/cmd_vel_flu', Twist, queue_size=10)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/rover_'+str(i)+'/cmd',String,queue_size=10)
    leader_cmd_vel_pub = rospy.Publisher("/xtdrone/leader/cmd_vel", Twist, queue_size=10)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader_cmd", String, queue_size=10)
    cmd= String()
    twist = Twist()    


    target_forward_vel   = 0.0
    target_leftward_vel   = 0.0
    target_upward_vel   = 0.0
    target_angular_vel  = 0.0
    control_forward_vel  = 0.0
    control_leftward_vel  = 0.0
    control_upward_vel  = 0.0
    control_angular_vel = 0.0

    count = 0

    try:
        print_msg()
        while(1):
            key = getKey()
            if key == 'w' :
                target_forward_vel = checkLinearLimitVelocity(target_forward_vel + LIN_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'x' :
                target_forward_vel = checkLinearLimitVelocity(target_forward_vel - LIN_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'a' :
                target_leftward_vel = checkLinearLimitVelocity(target_leftward_vel + LIN_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'd' :
                target_leftward_vel = checkLinearLimitVelocity(target_leftward_vel - LIN_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'i' :
                target_upward_vel = checkLinearLimitVelocity(target_upward_vel + LIN_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == ',' :
                target_upward_vel = checkLinearLimitVelocity(target_upward_vel - LIN_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'j':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'l':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                print_msg()
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
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
                target_forward_vel   = 0.0
                target_leftward_vel   = 0.0
                target_upward_vel   = 0.0
                target_angular_vel  = 0.0
                control_forward_vel  = 0.0
                control_leftward_vel  = 0.0
                control_upward_vel  = 0.0
                control_angular_vel = 0.0
                print_msg()
                print(vels(target_forward_vel,-target_leftward_vel,target_upward_vel,target_angular_vel))
            else:
                for i in range(10):
                    if key == str(i):
                        cmd = 'mission'+key
                        print_msg()
                        print(cmd)
                        cmd_vel_mask = True
                if (key == '\x03'):
                    break


            control_forward_vel = makeSimpleProfile(control_forward_vel, target_forward_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_leftward_vel = makeSimpleProfile(control_leftward_vel, target_leftward_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_upward_vel = makeSimpleProfile(control_upward_vel, target_upward_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_forward_vel; twist.linear.y = control_leftward_vel; twist.linear.z = control_upward_vel

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = control_angular_vel

            for i in range(rover_num):
                if ctrl_leader:
                    leader_cmd_vel_pub.publish(twist)
                    leader_cmd_pub.publish(cmd)
                else:
                    if not cmd_vel_mask:
                        multi_cmd_vel_flu_pub[i].publish(twist)    
                    multi_cmd_pub[i].publish(cmd)
                    
            cmd = ''
    except:
        print(e)

    finally:
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd = ''
        for i in range(rover_num):
                multi_cmd_vel_flu_pub[i].publish(twist)
                multi_cmd_pub[i].publish(cmd)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
