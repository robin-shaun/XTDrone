import rospy
import sys, select, os
import tty, termios
from std_msgs.msg import String, Float32


MAX_LIN_VEL = 100
MAX_ANGLE = 10
LIN_VEL_STEP_SIZE = 1
ANGLE_STEP_SIZE = 0.3

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
a/d : increase/decrease steering steering
i/, : no use
j/l : no use
r   : no use
t/y : no use
v/n : no use
b   : no use
s/k : stop
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
a/d : increase/decrease steering steering
i/, : no use
j/l : no use
r   : no use
t/y : no use
v/n : no use
b   : no use
s/k : stop
0~9 : extendable mission(eg.different formation configuration)
      this will mask the keyboard control
g   : control all drones
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

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    usv_num = int(sys.argv[1])
    rospy.init_node('usv_keyboard_control')
    multi_left_thrust_pub = [None]*usv_num
    multi_right_thrust_pub = [None]*usv_num
    for i in range(usv_num):
        multi_left_thrust_pub[i] = rospy.Publisher('/usv_'+str(i)+'/thrusters/left_thrust_cmd', Float32, queue_size=10)
        multi_right_thrust_pub[i] = rospy.Publisher('/usv_'+str(i)+'/thrusters/right_thrust_cmd', Float32, queue_size=10)
    leader_left_thrust_pub = rospy.Publisher("/usv/leader/left_thrust_cmd", Float32, queue_size=10)
    leader_right_thrust_pub = rospy.Publisher("/usv/leader/right_thrust_cmd", Float32, queue_size=10)
    left_thrust_cmd = Float32()
    right_thrust_cmd = Float32()

    forward  = 0.0
    steering  = 0.0



    print_msg()
    while(1):
        key = getKey()
        if key == 'w' :
            forward = forward + LIN_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward thrust %.2f\t steering thrust %.2f " % (forward, steering))
        elif key == 'x' :
            forward = forward - LIN_VEL_STEP_SIZE
            print_msg()
            print("currently:\t forward thrust %.2f\t steering thrust %.2f " % (forward, steering))
        elif key == 'a' :
            steering = steering + ANGLE_STEP_SIZE
            print_msg()
            print("currently:\t forward thrust %.2f\t steering thrust %.2f " % (forward, steering))
        elif key == 'd' :
            steering = steering - ANGLE_STEP_SIZE
            print_msg()
            print("currently:\t forward thrust %.2f\t steering thrust %.2f " % (forward, steering))

        elif key == 'g':
            ctrl_leader = not ctrl_leader
            print_msg()

        elif key == 's' :
            cmd_vel_mask = False
            forward   = 0.0
            steering   = 0.0
            print_msg()
            print("currently:\t forward thrust %.2f\t steering thrust %.2f " % (forward, steering))
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
        if steering > MAX_ANGLE:
            steering = MAX_ANGLE
        elif steering < -MAX_ANGLE:
            steering = -MAX_ANGLE

        left_thrust_cmd = (forward + steering) / 2
        right_thrust_cmd = (forward - steering) / 2
        
        for i in range(usv_num):
            if ctrl_leader:
                leader_left_thrust_pub.publish(left_thrust_cmd)
                leader_right_thrust_pub.publish(right_thrust_cmd)
            else:
                if not cmd_vel_mask:
                    multi_left_thrust_pub[i].publish(left_thrust_cmd)    
                    multi_right_thrust_pub[i].publish(right_thrust_cmd)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
