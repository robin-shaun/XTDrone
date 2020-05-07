import rospy
from geometry_msgs.msg import Pose
import sys, select, os
import tty, termios
from std_msgs.msg import String


MAX_LIN_VEL = 20
MAX_ANG_VEL = 0.1
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.01


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
v/n : AUTO.TAKEOFF/AUTO.LAND
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
v/n : AUTO.TAKEOFF/AUTO.LAND
b   : offboard
s   : loiter
k   : idle
0~9 : extendable mission(eg.different formation configuration)
g   : control all drones
o   : send setpoint
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
        
def vels(target_forward_vel, target_leftward_vel, target_upward_vel,target_orientation_vel):
    return "currently:\t forward x %s\t leftward y %s\t upward z %s\t orientation %s " % (target_forward_vel, target_leftward_vel, target_upward_vel, target_orientation_vel)

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

def checkpositionLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel

def checkorientationLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    plane_num = int(sys.argv[1])
    rospy.init_node('plane_keyboard_multi_control')
    multi_cmd_vel_flu_pub = [None]*plane_num
    multi_cmd_pub = [None]*plane_num
    for i in range(plane_num):
        multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/plane'+str(i)+'/cmd_pose_enu', Pose, queue_size=10)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/plane'+str(i)+'/cmd',String,queue_size=10)
    leader_cmd_vel_pub = rospy.Publisher("/xtdrone/leader/cmd_pose", Pose, queue_size=10)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader_cmd", String, queue_size=10)
    cmd= String()
    pose = Pose()    


    target_forward_vel   = 0.0
    target_leftward_vel   = 0.0
    target_upward_vel   = 0.0
    target_orientation_vel  = 0.0
    control_forward_vel  = 0.0
    control_leftward_vel  = 0.0
    control_upward_vel  = 0.0
    control_orientation_vel = 0.0

    count = 0

    print_msg()
    while(1):
        key = getKey()
        if key == 'w' :
            target_forward_vel = checkpositionLimitVelocity(target_forward_vel + LIN_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
        elif key == 'x' :
            target_forward_vel = checkpositionLimitVelocity(target_forward_vel - LIN_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
        elif key == 'a' :
            target_leftward_vel = checkpositionLimitVelocity(target_leftward_vel + LIN_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
        elif key == 'd' :
            target_leftward_vel = checkpositionLimitVelocity(target_leftward_vel - LIN_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
        elif key == 'i' :
            target_upward_vel = checkpositionLimitVelocity(target_upward_vel + LIN_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
        elif key == ',' :
            target_upward_vel = checkpositionLimitVelocity(target_upward_vel - LIN_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
        elif key == 'j':
            target_orientation_vel = checkorientationLimitVelocity(target_orientation_vel + ANG_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
        elif key == 'l':
            target_orientation_vel = checkorientationLimitVelocity(target_orientation_vel - ANG_VEL_STEP_SIZE)
            print_msg()
            print(vels(target_forward_vel,target_leftward_vel,target_upward_vel,target_orientation_vel))
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
        else:
            for i in range(10):
                if key == str(i):
                    cmd = 'mission'+key
                    print_msg()
                    print(cmd)
            if (key == '\x03'):
                break


        control_forward_vel = makeSimpleProfile(control_forward_vel, target_forward_vel, (LIN_VEL_STEP_SIZE/2.0))
        control_leftward_vel = makeSimpleProfile(control_leftward_vel, target_leftward_vel, (LIN_VEL_STEP_SIZE/2.0))
        control_upward_vel = makeSimpleProfile(control_upward_vel, target_upward_vel, (LIN_VEL_STEP_SIZE/2.0))
        pose.position.x = control_forward_vel; pose.position.y = control_leftward_vel; pose.position.z = control_upward_vel

        control_orientation_vel = makeSimpleProfile(control_orientation_vel, target_orientation_vel, (ANG_VEL_STEP_SIZE/2.0))
        pose.orientation.x = 0.0; pose.orientation.y = 0.0;  pose.orientation.z = control_orientation_vel   
        for i in range(plane_num):
            if ctrl_leader:
                if send_flag:
                    leader_cmd_vel_pub.publish(pose)
                leader_cmd_pub.publish(cmd)
            else:
                if send_flag:
                    multi_cmd_vel_flu_pub[i].publish(pose)    
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
