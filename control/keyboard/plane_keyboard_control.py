import rospy
from geometry_msgs.msg import Pose
import sys, select, os
import tty, termios
from std_msgs.msg import String

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
j/l : increase/decrease yaw
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
j/l : increase/decrease yaw
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

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    plane_num = int(sys.argv[1])
    rospy.init_node('plane_keyboard_control')
    multi_cmd_pose_enu_pub = [None]*plane_num
    multi_cmd_pub = [None]*plane_num
    for i in range(plane_num):
        multi_cmd_pose_enu_pub[i] = rospy.Publisher('/xtdrone/plane_'+str(i)+'/cmd_pose_enu', Pose, queue_size=1)
        multi_cmd_pub[i] = rospy.Publisher('/xtdrone/plane_'+str(i)+'/cmd',String,queue_size=1)
    leader_pose_pub = rospy.Publisher("/xtdrone/leader/cmd_pose", Pose, queue_size=1)
    leader_cmd_pub = rospy.Publisher("/xtdrone/leader_cmd", String, queue_size=3)
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
        else:
            for i in range(10):
                if key == str(i):
                    cmd = 'mission'+key
                    print_msg()
                    print(cmd)
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
