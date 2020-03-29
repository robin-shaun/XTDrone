#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import TwistStamped, Twist, Vector3, PoseStamped
from std_msgs.msg import String 
from pyquaternion import Quaternion
import time
import math
import numpy 
import sys

class Leader:

    def __init__(self, leader_id, uav_num):
        self.hover = True
        self.id = leader_id
        self.local_pose = PoseStamped()
        self.local_velocity = TwistStamped()
        self.cmd_vel_enu = Twist()
        self.cmd_accel_enu = Vector3()
        self.follower_num = uav_num - 1
        self.followers_info = [None]*self.follower_num
        self.follower_arrived_num = 0
        self.follower_all_arrived = True
        self.avoid_vel = Vector3(0,0,0)
        self.formation_config = 'waiting'
        self.followers_receive = [False]*self.follower_num
        self.target_height_recorded = False
        self.f = 200
        self.K_vxy = 0.5
        self.K_vz = 0.5
        self.K_pz = 0.5
        self.local_pose_sub = rospy.Subscriber("/uav"+str(self.id)+"/mavros/local_position/pose", PoseStamped , self.local_pose_callback)
        self.local_velocity_sub = rospy.Subscriber("/uav"+str(self.id)+"/mavros/local_position/velocity_local", TwistStamped , self.local_velocity_callback)
        self.cmd_vel_sub = rospy.Subscriber("/xtdrone/leader/cmd_vel", Twist, self.cmd_vel_callback)
        self.avoid_vel_sub = rospy.Subscriber("/xtdrone/uav"+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback)
        self.formation_switch_sub = rospy.Subscriber("/gcs/formation_switch",String, self.cmd_callback)
        for i in range(self.follower_num):
            rospy.Subscriber('/xtdrone/uav'+str(i+1)+'/info',String,self.followers_info_callback,i)
        self.local_pose_pub = rospy.Publisher("/xtdrone/leader/pose", PoseStamped , queue_size=10)
        self.formation_switch_pub = rospy.Publisher("/xtdrone/formation_switch",String, queue_size=10)
        self.accel_enu_pub =  rospy.Publisher('/xtdrone/uav'+str(self.id)+'/cmd_accel_enu', Vector3, queue_size=10)

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def local_velocity_callback(self, msg):
        self.local_velocity = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_enu = msg
        if msg.linear.z == 0:
            self.hover = True
        else:
            self.hover = False

    def cmd_callback(self, msg):
        if not msg.data == '': 
            self.formation_config = msg.data
            #print("Switch to Formation"+self.formation_config)

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg
        #print('leader: ', self.avoid_vel)
    
    def followers_info_callback(self, msg, id):
        self.followers_info[id] = msg.data
        #print("follower"+str(id)+":"+ msg.data) 

    def loop(self):
        rospy.init_node('leader')
        rate = rospy.Rate(self.f)
        while True:
            #self.cmd_vel_enu = Twist()
            for i in range(self.follower_num):
                if self.followers_info[i] == "Received":
                    self.followers_receive[i] = True
            if self.follower_arrived_num > self.follower_num - 1:
                self.follower_all_arrived = True
            if self.follower_all_arrived:
                self.formation_switch_pub.publish(self.formation_config)
            if self.formation_config == 'pyramid':
                if not self.target_height_recorded:
                    target_height =  self.local_pose.pose.position.z + 2
                    self.target_height_recorded = True
                self.cmd_vel_enu.linear.z = self.K_vz * (target_height - self.local_pose.pose.position.z)
            self.cmd_vel_enu.linear.x += self.avoid_vel.x
            self.cmd_vel_enu.linear.y += self.avoid_vel.y
            self.cmd_vel_enu.linear.z += self.avoid_vel.z
            self.cmd_accel_enu.x = self.K_vxy * (self.cmd_vel_enu.linear.x - self.local_velocity.twist.linear.x)
            self.cmd_accel_enu.y = self.K_vxy * (self.cmd_vel_enu.linear.y - self.local_velocity.twist.linear.y)
            self.cmd_accel_enu.y = self.K_vz * (self.cmd_vel_enu.linear.z - self.local_velocity.twist.linear.z)
            self.accel_enu_pub.publish(self.cmd_accel_enu)
            self.local_pose_pub.publish(self.local_pose)
            rate.sleep()

if __name__ == '__main__':
    leader = Leader(1,int(sys.argv[1]))
    leader.loop()   
