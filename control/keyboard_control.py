import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String

MAX_LIN_VEL = 5
MAX_ANG_VEL = 0.1
LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.01

msg = """
Control Your XTDrone!
---------------------------
    1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d               j    k    l
        x       v    b   n        ,

w/x : increase/decrease forward velocity (-1~1)
a/d : increase/decrease leftward velocity (-1~1)
i/, : increase/decrease upward velocity (-1~1)
j/l : increase/decrease angular velocity (-0.1~0.1)
r   : return home
t/y : arm/disarm
v   : mission
n   : land
b   : offboard
s or k : hover
0~9 : extendable mission
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

    rospy.init_node('keyboard_control')
    cmd_vel_flu_pub = rospy.Publisher('/xtdrone/cmd_vel_flu', Twist, queue_size=10)
    cmd_pub = rospy.Publisher('/xtdrone/cmd',String,queue_size=10)
    cmd = String()

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
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_forward_vel = checkLinearLimitVelocity(target_forward_vel + LIN_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'x' :
                target_forward_vel = checkLinearLimitVelocity(target_forward_vel - LIN_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'a' :
                target_leftward_vel = checkLinearLimitVelocity(target_leftward_vel + LIN_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'd' :
                target_leftward_vel = checkLinearLimitVelocity(target_leftward_vel - LIN_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'i' :
                target_upward_vel = checkLinearLimitVelocity(target_upward_vel + LIN_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == ',' :
                target_upward_vel = checkLinearLimitVelocity(target_upward_vel - LIN_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'j':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'l':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                print(msg)
                print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_angular_vel))
            elif key == 'r':
                cmd = 'AUTO.RTL'
                print(msg)
                print('Returning home')
            elif key == 't':
                cmd = 'ARM'
                print(msg)
                print('Arming')
            elif key == 'y':
                cmd = 'DISARM'
                print(msg)
                print('Disarming')
            elif key == 'v':
                cmd = 'AUTO.MiSSION'
                print_msg()
                print('Mission')
            elif key == 'b':
                cmd = 'OFFBOARD'
                print(msg)
                print('Offboard')
            elif key == 'n':
                cmd = 'AUTO.LAND'
                print(msg)
                print('Landing')
            elif key == 's' or key == 'k' :
                target_forward_vel   = 0.0
                target_leftward_vel   = 0.0
                target_upward_vel   = 0.0
                target_angular_vel  = 0.0
                control_forward_vel  = 0.0
                control_leftward_vel  = 0.0
                control_upward_vel  = 0.0
                control_angular_vel = 0.0
                print(msg)
                print(vels(target_forward_vel,-target_leftward_vel,target_upward_vel,target_angular_vel))
            else:
                if (key == '\x03'):
                    break

            twist = Twist()

            control_forward_vel = makeSimpleProfile(control_forward_vel, target_forward_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_leftward_vel = makeSimpleProfile(control_leftward_vel, target_leftward_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_upward_vel = makeSimpleProfile(control_upward_vel, target_upward_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_forward_vel; twist.linear.y = control_leftward_vel; twist.linear.z = control_upward_vel

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = control_angular_vel

            cmd_vel_flu_pub.publish(twist)
            cmd_pub.publish(cmd)
            cmd = ''
    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_vel_flu_pub.publish(twist)
        cmd_pub.publish(cmd)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
