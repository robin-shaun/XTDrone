#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String 
from pyquaternion import Quaternion
import time
import math
import numpy 
import sys
if sys.argv[2] == '6':
    from formation_dict import formation_dict_6 as formation_dict
elif sys.argv[2] == '9':
    from formation_dict import formation_dict_9 as formation_dict

class Leader:

    def __init__(self, uav_type, leader_id, uav_num):
        self.hover = True
        self.id = leader_id
        self.local_pose = PoseStamped()
        self.cmd_vel_enu = Twist()
        self.follower_num = uav_num - 1
        self.followers_info = ["Moving"]*self.follower_num
        self.follower_arrived_num = 0
        self.follower_all_arrived = True
        self.avoid_accel = Vector3(0,0,0)
        self.formation_config = 'waiting'
        self.target_height_recorded = False
        self.cmd = String()
        self.f = 200
        self.Kz = 0.5
        self.local_pose_sub = rospy.Subscriber(uav_type+'_'+str(self.id)+"/mavros/local_position/pose", PoseStamped , self.local_pose_callback)
        self.cmd_vel_sub = rospy.Subscriber("/xtdrone/leader/cmd_vel_flu", Twist, self.cmd_vel_callback)
        self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+uav_type+'_'+str(self.id)+"/avoid_accel", Vector3, self.avoid_accel_callback)
        self.leader_cmd_sub = rospy.Subscriber("/xtdrone/leader/cmd",String, self.cmd_callback)

        for i in range(self.follower_num):
            rospy.Subscriber('/xtdrone/'+uav_type+'_'+str(i+1)+'/info',String,self.followers_info_callback,i)
        self.local_pose_pub = rospy.Publisher("/xtdrone/leader/pose", PoseStamped , queue_size=10)
        self.formation_switch_pub = rospy.Publisher("/xtdrone/formation_switch",String, queue_size=10)
        self.vel_enu_pub =  rospy.Publisher('/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd_vel_enu', Twist, queue_size=10)
        self.cmd_pub = rospy.Publisher('/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd', String, queue_size=10)

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_enu = msg
        if msg.linear.z == 0:
            self.hover = True
        else:
            self.hover = False

    def cmd_callback(self, msg):
        if msg.data in formation_dict.keys():
            self.formation_config = msg.data
        else:
            self.cmd = msg.data

    def avoid_accel_callback(self, msg):
        self.avoid_accel = msg
    
    def followers_info_callback(self, msg, id):
        self.followers_info[id] = msg.data
        #print("follower"+str(id)+":"+ msg.data) 

    def loop(self):
        rospy.init_node('leader')
        rate = rospy.Rate(self.f)
        while True:
            #self.cmd_vel_enu = Twist()
            for follower_info in self.followers_info:
                if follower_info == "Arrived":
                    self.follower_arrived_num += 1
            if self.follower_arrived_num > self.follower_num - 1:
                self.follower_all_arrived = True
            if self.follower_all_arrived:
                self.formation_switch_pub.publish(self.formation_config)
            if self.formation_config == 'pyramid':
                if not self.target_height_recorded:
                    target_height =  self.local_pose.pose.position.z + 2
                    self.target_height_recorded = True
                self.cmd_vel_enu.linear.z = self.Kz * (target_height - self.local_pose.pose.position.z)
            self.cmd_vel_enu.linear.x += self.avoid_accel.x
            self.cmd_vel_enu.linear.y += self.avoid_accel.y
            self.cmd_vel_enu.linear.z += self.avoid_accel.z
            self.vel_enu_pub.publish(self.cmd_vel_enu)
            self.local_pose_pub.publish(self.local_pose)
            self.cmd_pub.publish(self.cmd)
            rate.sleep()

if __name__ == '__main__':
    leader = Leader(sys.argv[1], 0, int(sys.argv[2]))
    leader.loop()   
