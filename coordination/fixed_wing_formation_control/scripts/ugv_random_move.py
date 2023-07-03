#! /usr/bin/env python2
#-*- coding: UTF-8 -*-

from numpy.lib.function_base import angle
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String
import numpy as np
import threading
import time


MAX_LIN_VEL = 10
MAX_ANGLE = 1
 
forward_vel = 0  #全局的线速度指令
angle_vel = 0  #全局的角速度指令

cmd_vel_mask = False
ctrl_leader = False

msg2all = """
-----------------------------------------------------
Control ugv random move
s: start ugv
e:end ugv
CRTL-C TO QUIT
-----------------------------------------------------
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
a/d : increase/decrease steering angle
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

def change_forward_vel():
    global forward_vel
    forward_vel = np.random.uniform(3, 6)  #随机产生新的速度指令 线速度的变化可以快一点，可以在这里边进一步设置速度的变化
    # print_msg()
    print("currently:\t forward_vel %.2f\t angle_vel %.2f \n" % (forward_vel, angle_vel))

    t1= threading.Timer(60, change_forward_vel)
    t1.start()

def change_angle_vel():
    global angle_vel
    angle_vel = np.random.uniform(-1,1) #产生新的角速度指令 角速度的指令就慢一点， 
    # print_msg()
    print("currently:\t forward_vel %.2f\t angle_vel %.2f \n" % (forward_vel, angle_vel))

    #新的角速度指令持续一段时间，然后归零，这是为了避免原地转圈的结果
    time.sleep(5)
    angle_vel = 0
    # print_msg()
    print("currently:\t forward_vel %.2f\t angle_vel %.2f \n" % (forward_vel, angle_vel))

    t2 = threading.Timer(90, change_angle_vel)
    t2.start()

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    ugv_num = int(sys.argv[1])
    rospy.init_node('ugv_keyboard_multi_control')
    multi_cmd_vel_flu_pub = [None]*ugv_num
    multi_cmd_pub = [None]*ugv_num
    for i in range(ugv_num):
        multi_cmd_vel_flu_pub[i] = rospy.Publisher('/ugv_'+str(i)+'/cmd_vel', Twist, queue_size=10)
        multi_cmd_pub[i] = rospy.Publisher('/ugv_'+str(i)+'/cmd',String,queue_size=10)

    leader_cmd_vel_pub = rospy.Publisher("/ugv/leader/cmd_vel", Twist, queue_size=10)
    leader_cmd_pub = rospy.Publisher("/ugv/leader_cmd", String, queue_size=10)

    cmd= String()
    twist = Twist()    

    #main函数中声明的变量是默认为全局的
    t1 = threading.Timer(60, change_forward_vel)
    t1.start()
    t2 = threading.Timer(90, change_angle_vel)  
    t2.start()

    print_msg()
    while(1):
        key = getKey()

        if key == 's':
            forward_vel = 5  #设置了起始时刻的速度不为0, 会先以当前速度运行两分钟之后在改变
            angle_vel = 0
            # print_msg()
            print("currently:\t forward_vel %.2f\t angle_vel %.2f \n" % (forward_vel, angle_vel))


        elif key == 'e':
            forward_vel = 0
            angle_vel = 0
            # print_msg()
            print("currently:\t forward_vel %.2f\t angle_vel %.2f \n" % (forward_vel, angle_vel))

        else:
            if (key ==  '\x03'):
                break      

        twist.linear.x = forward_vel; twist.angular.z =  angle_vel
        
        for i in range(ugv_num):
            if ctrl_leader:
                leader_cmd_vel_pub.publish(twist)
                leader_cmd_pub.publish(cmd)
            else:
                if not cmd_vel_mask:
                    multi_cmd_vel_flu_pub[i].publish(twist)    
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
