#!/usr/bin/python
# -*- coding: UTF-8 -*-
### This code is about the distributed formation control with a consensus protocol
### For more details, please see the paper on https://arxiv.org/abs/2005.01125

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TwistStamped
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
import sys
import numpy

class Follower:
    def __init__(self, uav_type, uav_id, uav_num, control_type):
        self.uav_type = uav_type
        self.id = uav_id
        self.uav_num = uav_num
        self.control_type = control_type
        self.f = 30
        self.formation_pattern = None
        self.communication_topology = None
        self.changed_id = numpy.arange(0, self.uav_num-1)
        self.local_leader_ids = []  
        self.Kp = 1.0
        self.Kp_avoid = 1.0
        self.weight = 1.0
        self.pose = [PoseStamped() for i in range(self.uav_num)]
        rospy.init_node('follower' + str(self.id - 1))
        self.pose_sub = [[] for i in range(self.uav_num)] 
        self.formation_pattern_sub = rospy.Subscriber("/xtdrone/formation_pattern", Float32MultiArray, self.formation_pattern_callback, queue_size=1)
        self.communication_topology_sub = rospy.Subscriber("/xtdrone/communication_topology", Int32MultiArray, self.communication_topology_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/xtdrone/' + self.uav_type + '_' + str(self.id) + '/cmd', String, queue_size=1)
        for i in range(self.uav_num):
            self.pose_sub[i] = rospy.Subscriber(
                self.uav_type + '_' + str(i) + "/mavros/local_position/pose", PoseStamped,
                self.pose_callback, i, queue_size=1)
        self.formation_pattern = None
        if self.control_type == "vel":
            self.vel_max = 1
            self.cmd_vel_enu = Twist()
            self.avoid_vel = Vector3()
            self.avoid_vel_sub = rospy.Subscriber("/xtdrone/" + self.uav_type + '_' + str(self.id) + "/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)
            self.vel_enu_pub = rospy.Publisher('/xtdrone/' + self.uav_type + '_' + str(self.id) + '/cmd_vel_enu', Twist, queue_size=1)
        elif self.control_type == "accel":
            self.accel_max = 1
            self.gamma = (4.0 / self.Kp) ** 0.5
            self.cmd_accel_enu = Twist()
            self.avoid_accel = Vector3()
            self.avoid_accel_sub = rospy.Subscriber("/xtdrone/" + self.uav_type + '_' + str(self.id) + "/avoid_accel", Vector3, self.avoid_accel_callback, queue_size=1)
            self.accel_enu_pub = rospy.Publisher('/xtdrone/' + self.uav_type + '_' + str(self.id) + '/cmd_accel_enu', Twist, queue_size=1)
            self.velocity = [TwistStamped() for i in range(self.uav_num)]
            self.velocity_sub = [[] for i in range(self.uav_num)]
            for i in range(self.uav_num):
                self.velocity_sub[i] = rospy.Subscriber(
                self.uav_type + '_' + str(i) + "/mavros/local_position/velocity_local", TwistStamped,
                self.velocity_callback, i, queue_size=1)
        else:
            print("Only vel and accel are supported.")

    def pose_callback(self, msg, id):
        self.pose[id] = msg
    
    def velocity_callback(self, msg, id):
        self.velocity[id] = msg

    def formation_pattern_callback(self, msg):
        self.formation_pattern = numpy.array(msg.data).reshape(3, self.uav_num-1) 

    def communication_topology_callback(self, msg):
        self.communication_topology = numpy.array(msg.data).reshape(self.uav_num, self.uav_num)
        self.local_leader_ids = numpy.argwhere(self.communication_topology[self.id, :] == 1)
        self.local_leader_ids = self.local_leader_ids.reshape(self.local_leader_ids.shape[0])
        self.weight = self.Kp / len(self.local_leader_ids)

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    def avoid_accel_callback(self, msg):
        self.avoid_accel = msg

    def loop(self):
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if (not self.communication_topology is None and not self.formation_pattern is None):
                input = Vector3(0, 0, 0)
                if (self.control_type == "vel"):
                    for local_leader_id in self.local_leader_ids:
                        if local_leader_id == 0: # The local leader is the global leader
                            input.x += self.pose[local_leader_id].pose.position.x - (self.pose[self.id].pose.position.x -  self.formation_pattern[0, self.id - 1])
                            input.y += self.pose[local_leader_id].pose.position.y - (self.pose[self.id].pose.position.y -  self.formation_pattern[1, self.id - 1])
                            input.z += self.pose[local_leader_id].pose.position.z - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.id - 1])
                        else:
                            input.x += (self.pose[local_leader_id].pose.position.x - self.formation_pattern[0, local_leader_id - 1]) - (self.pose[self.id].pose.position.x -  self.formation_pattern[0, self.id - 1])
                            input.y += (self.pose[local_leader_id].pose.position.y - self.formation_pattern[1, local_leader_id - 1]) - (self.pose[self.id].pose.position.y -  self.formation_pattern[1, self.id - 1])
                            input.z += (self.pose[local_leader_id].pose.position.z - self.formation_pattern[2, local_leader_id - 1]) - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.id - 1])
                        # This is used to check the steady state error
                        # if (self.id == 1):
                            # print(self.pose[local_leader_id].pose.position.x - self.pose[self.id].pose.position.x)
                    
                    self.cmd_vel_enu.linear.x = self.weight * input.x + self.Kp_avoid * self.avoid_vel.x
                    self.cmd_vel_enu.linear.y = self.weight * input.y + self.Kp_avoid * self.avoid_vel.y
                    self.cmd_vel_enu.linear.z = self.weight * input.z + self.Kp_avoid * self.avoid_vel.z 
                    cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5
                    if (cmd_vel_magnitude > 3**0.5 * self.vel_max):
                        self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                        self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                        self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max

                    self.vel_enu_pub.publish(self.cmd_vel_enu)
                    
                elif (self.control_type == "accel"):  
                    for local_leader_id in self.local_leader_ids:
                        if local_leader_id == 0: # The local leader is the global leader
                            input.x += self.pose[local_leader_id].pose.position.x - self.pose[self.id].pose.position.x + \
                                self.formation_pattern[0, self.id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.x - self.velocity[self.id].twist.linear.x)
                            input.y += self.pose[local_leader_id].pose.position.y - self.pose[self.id].pose.position.y + \
                                self.formation_pattern[1, self.id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.y - self.velocity[self.id].twist.linear.y)
                            input.z += self.pose[local_leader_id].pose.position.z - self.pose[self.id].pose.position.z + \
                                self.formation_pattern[2, self.id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.z - self.velocity[self.id].twist.linear.z)
                        else:
                            input.x += self.pose[local_leader_id].pose.position.x - self.pose[self.id].pose.position.x + \
                                self.formation_pattern[0, self.id - 1] - self.formation_pattern[0, local_leader_id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.x - self.velocity[self.id].twist.linear.x)
                            input.y += self.pose[local_leader_id].pose.position.y - self.pose[self.id].pose.position.y + \
                                self.formation_pattern[1, self.id - 1] - self.formation_pattern[1, local_leader_id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.y - self.velocity[self.id].twist.linear.y)
                            input.z += self.pose[local_leader_id].pose.position.z - self.pose[self.id].pose.position.z + \
                                self.formation_pattern[2, self.id - 1] - self.formation_pattern[2, local_leader_id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.z - self.velocity[self.id].twist.linear.z)
                        # This is used to check the steady state error
                        # if (self.id == 1):
                        #     print(self.pose[local_leader_id].pose.position.x - self.pose[self.id].pose.position.x)

                    self.cmd_accel_enu.linear.x = self.weight * input.x + self.Kp_avoid * self.avoid_accel.x
                    self.cmd_accel_enu.linear.y = self.weight * input.y + self.Kp_avoid * self.avoid_accel.y
                    self.cmd_accel_enu.linear.z = self.weight * input.z + self.Kp_avoid * self.avoid_accel.z
                    cmd_accel_magnitude = (self.cmd_accel_enu.linear.x**2 + self.cmd_accel_enu.linear.y**2 + self.cmd_accel_enu.linear.z**2)**0.5
                    if (cmd_accel_magnitude > 3**0.5 * self.accel_max):
                        self.cmd_accel_enu.linear.x = self.cmd_accel_enu.linear.x / cmd_accel_magnitude * self.accel_max
                        self.cmd_accel_enu.linear.y = self.cmd_accel_enu.linear.y / cmd_accel_magnitude * self.accel_max
                        self.cmd_accel_enu.linear.z = self.cmd_accel_enu.linear.z / cmd_accel_magnitude * self.accel_max

                    self.accel_enu_pub.publish(self.cmd_accel_enu)

            rate.sleep()

if __name__ == '__main__':
    follower = Follower(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])
    follower.loop()
