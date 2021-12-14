#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String, Float32MultiArray
import sys
import numpy

class Follower:

    def __init__(self, uav_type, uav_id, uav_num):
        self.hover = "HOVER"
        self.uav_type = uav_type
        self.uav_num = uav_num
        self.id = uav_id
        self.f = 30 
        self.pose = PoseStamped()
        self.cmd_vel_enu = Twist()
        self.avoid_vel = Vector3()
        self.formation_pattern = None
        self.Kp = 1.0 
        self.Kp_avoid = 2.0
        self.vel_max = 1.0
        self.leader_pose = PoseStamped()

        self.pose_sub = rospy.Subscriber(self.uav_type+'_'+str(self.id)+"/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+self.uav_type+'_'+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)
        self.formation_pattern_sub = rospy.Subscriber("/xtdrone/formation_pattern", Float32MultiArray, self.formation_pattern_callback, queue_size=1)

        self.vel_enu_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd_vel_enu', Twist, queue_size=1)
        self.info_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/info', String, queue_size=1)
        self.cmd_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd',String,queue_size=1)
        self.leader_pose_sub = rospy.Subscriber(self.uav_type+"_0/mavros/local_position/pose", PoseStamped, self.leader_pose_callback, queue_size=1)

    def formation_pattern_callback(self, msg):
        self.formation_pattern = numpy.array(msg.data).reshape(3, self.uav_num-1) 

    def pose_callback(self, msg):
        self.pose = msg

    def leader_pose_callback(self, msg):
        self.leader_pose = msg    

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    def loop(self):
        rospy.init_node('follower'+str(self.id-1))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if (not self.formation_pattern is None):
                self.cmd_vel_enu.linear.x = self.Kp * ((self.leader_pose.pose.position.x + self.formation_pattern[0, self.id - 1]) - self.pose.pose.position.x)
                self.cmd_vel_enu.linear.y = self.Kp * ((self.leader_pose.pose.position.y + self.formation_pattern[1, self.id - 1]) - self.pose.pose.position.y) 
                self.cmd_vel_enu.linear.z = self.Kp * ((self.leader_pose.pose.position.z + self.formation_pattern[2, self.id - 1]) - self.pose.pose.position.z) 
                self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.Kp_avoid * self.avoid_vel.x
                self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.Kp_avoid * self.avoid_vel.y
                self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.Kp_avoid * self.avoid_vel.z
                cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5 
                if cmd_vel_magnitude > 3**0.5 * self.vel_max:
                    self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max
                
                self.vel_enu_pub.publish(self.cmd_vel_enu)

            rate.sleep()

if __name__ == '__main__':
    follower = Follower(sys.argv[1],int(sys.argv[2]), int(sys.argv[3]))
    follower.loop()   