# -*- coding: UTF-8 -*-
"""
Framework for Formation GCS

* send cmd by ros topic

Before running this code, you need to make sure ros master started:
if not:
$ roscore    # start ros master

And then, you can run this code via:
$ python formation_gcs.py            # start gcs
"""

import rospy
from std_msgs.msg import String
import sys, select, os
import tty, termios
import copy
from enviroment import Env
from ga2 import GA2
from geometry_msgs.msg import PoseStamped
import math

msg2ui = """
Welcome use gcs!
Let's go!
---------------------------
   1   2   3   4   5   6   7   8   9   0
                     t             
                         h             l

t  :  takeoff
l  :  land
h  :  hover
0  :  search target
1  :  formation 1
2  :  fortion 2
3~9 : extendable mission(eg.different formation configuration)
q/Q : quit
"""

uav_type = 'typhoon_h480'

uav_bias = [[0,0,0],[0,3,0],[0,-3,0]]

class GroundControler:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.gcs_cmd = String()
        self.gcs_cmd = 'start'
        self.formation_mode = {'FORM_0':'search', 'FORM_1':'straight', 'FORM_2':'triangle'}
        # ros publishers
        self.gcs_cmd_pub = rospy.Publisher("/formation_gcs/cmd", String, queue_size=10)

        #ga
        self.map_size_x = 80
        self.map_size_y = 80
        self.map_size = [self.map_size_x, self.map_size_y]
        self.vehicle_num = 3
        self.target_num = 10
        self.target = [PoseStamped() for i in range(self.vehicle_num)]
        self.env = Env(self.vehicle_num, self.target_num, self.map_size, False)
        self.ga2_result = None
        self.ga2 = GA2(self.vehicle_num, self.env.vehicles_speed, self.target_num, self.env.targets, self.env.time_limit)
        self.ga_flag = False
        self.pos_i = [1 for i in range(self.vehicle_num)]

        # ros subscribers
        for i in range(self.vehicle_num):
            rospy.Subscriber(uav_type+ '_' + str(i) + "/mavros/local_position/pose", PoseStamped, self.local_pose_callback, i, queue_size = 2)

        # ros publishers
        self.local_target_pub = [rospy.Publisher(uav_type + '_' + str(i) + '/mavros/setpoint_position/local', PoseStamped, queue_size=10) for i in range(self.vehicle_num)]

        self.global_pose = [PoseStamped() for i in range(self.vehicle_num)]

        print("Ground Controller Start!")

    def working(self):
        rospy.init_node("gcs_control_node")
        rate = rospy.Rate(60)
        self.print_ui()
        while rospy.is_shutdown() is False:
            key = self.getKey()
            if key == 't' :
                self.gcs_cmd = 'TAKEOFF'
                self.print_ui()
                print(self.gcs_cmd)
                self.gcs_cmd_pub.publish(self.gcs_cmd)

            elif key == 'l':
                self.gcs_cmd = 'AUTO.LAND'
                self.print_ui()
                print(self.gcs_cmd)
                self.gcs_cmd_pub.publish(self.gcs_cmd)

            elif key == 'h':
                self.gcs_cmd = 'HOVOR'
                self.print_ui()
                print(self.gcs_cmd)
                self.gcs_cmd_pub.publish(self.gcs_cmd)

            else:
                if key == '0':
                    if not self.ga_flag:
                        self.ga2_result = self.ga2.run()[0]
                        # print(self.ga2_result[0])
                        # print(self.env.targets[self.ga2_result[0]])
                        # for uav_i in range(self.vehicle_num):
                        #     for i in range(len(self.ga2_result[uav_i])):
                        #         print(self.env.targets[self.ga2_result[uav_i]][i][0],self.env.targets[self.ga2_result[uav_i]][i][1])
                        self.ga_flag = True
                    self.gcs_cmd = 'FORM_' + key
                    self.print_ui()
                    print(self.formation_mode[self.gcs_cmd])
                    self.gcs_cmd_pub.publish(self.gcs_cmd)

                for i in range(2):
                    if key == str(i+1):
                        self.gcs_cmd = 'FORM_'+key
                        self.print_ui()
                        print(self.formation_mode[self.gcs_cmd])
                        self.gcs_cmd_pub.publish(self.gcs_cmd)

                if key == 'q' or key =='Q':
                    break
            if self.ga_flag:
                for uav_i in range(self.vehicle_num):
                    if uav_i == 1:
                        print(self.pos_i[uav_i],self.env.targets[self.ga2_result[uav_i]][self.pos_i[uav_i]])
                    if self.is_get_target(uav_i,self.env.targets[self.ga2_result[uav_i]][self.pos_i[uav_i]][0],self.env.targets[self.ga2_result[uav_i]][self.pos_i[uav_i]][1]):
                        self.pos_i[uav_i] += 1
                        if self.pos_i[uav_i] < len(self.ga2_result[uav_i]):
                            self.target[uav_i] = self.construct_target(self.env.targets[self.ga2_result[uav_i]][self.pos_i[uav_i]][0],self.env.targets[self.ga2_result[uav_i]][self.pos_i[uav_i]][1])
                        else:
                            self.pos_i[uav_i] -= 1
                        # print(self.pos_i[uav_i])
                    self.local_target_pub[uav_i].publish(self.target[uav_i])
                    print(self.target[uav_i])

            rate.sleep()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def construct_target(self,x,y):
        target_pos = PoseStamped()
        target_pos.pose.position.x = x
        target_pos.pose.position.y = y
        target_pos.pose.position.z = 6
        print(target_pos)
        return target_pos

    def is_get_target(self,i,x,y):
        if math.sqrt((self.global_pose[i].pose.position.x-x)*(self.global_pose[i].pose.position.x-x)+\
                     (self.global_pose[i].pose.position.y-y)*(self.global_pose[i].pose.position.y-y))< 0.5:
            return True
        else:
            return False

    def local_pose_callback(self, msg, i):
        self.global_pose[i] = copy.deepcopy(msg)
        self.global_pose[i].pose.position.x += uav_bias[i][0]
        self.global_pose[i].pose.position.y += uav_bias[i][1]
        self.global_pose[i].pose.position.z += uav_bias[i][2]

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_ui(self):
            print(msg2ui)

if __name__ == '__main__':
    groundcontroler = GroundControler()
    groundcontroler.working()