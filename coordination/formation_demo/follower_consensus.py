#!/usr/bin/python
# -*- coding: UTF-8 -*-
### This code is about the distributed formation control of consensus protocol with a certain
### Laplacian matrix and the formation transformation based on a task allocation algorithm——
### KM for the shorest distances of all the UAVs to achieve the new pattern.
### For more information of these two algorithms, please see the paper on https://arxiv.org/abs/2005.01125

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TwistStamped
from std_msgs.msg import String, Int64MultiArray
import sys
import numpy

# # formation patterns
# if sys.argv[3] == '6':
#     from formation_dict import formation_dict_6 as formation_dict
# elif sys.argv[3] == '9':
#     from formation_dict import formation_dict_9 as formation_dict
# elif sys.argv[3] == '18':
#     from formation_dict import formation_dict_18 as formation_dict
# else:
#     print("Only 6, 9 and 18 UAVs are supported.")

class Follower:
    def __init__(self, uav_type, uav_id, uav_num, control_type):
        self.uav_type = uav_type
        self.id = uav_id
        self.uav_num = uav_num
        self.control_type = control_type
        self.f = 30  # control and communication rate
        # self.formation_config = 'waiting'
        self.formation_pattern = None
        self.changed_id = numpy.arange(0, self.uav_num-1)
        self.local_leader_ids = []  
        self.local_leader_count = 1  
        self.Kp = 1.0
        self.weight = 1.0
        self.L_matrix = None
        self.wait_cmd = 'HOVER'
        self.pose = [PoseStamped() for i in range(self.uav_num)]
        rospy.init_node('follower' + str(self.id - 1))
        self.pose_sub = [[] for i in range(self.uav_num)]
        # self.formation_switch_sub = rospy.Subscriber("/xtdrone/formation_switch", String, self.formation_switch_callback, queue_size=1) 
        self.formation_pattern_sub = rospy.Subscriber("/xtdrone/formation_pattern", Int64MultiArray, self.formation_pattern_callback, queue_size=1)
        self.changed_id_sub = rospy.Subscriber("/xtdrone/changed_id", Int64MultiArray, self.changed_id_callback, queue_size=1)
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

    def changed_id_callback(self, msg):
        self.changed_id = numpy.array(msg.data)   

    def formation_pattern_callback(self, msg):
        self.formation_pattern = numpy.array(msg.data) 

    def get_new_formation(self, changed_id, change_formation):
        new_formation = numpy.zeros((3, self.uav_num - 1))
        position = numpy.zeros((3, self.uav_num - 1))
        changed_id = [i + 1 for i in changed_id]
        for i in range(0, self.uav_num - 1):
            position[:, i] = change_formation[:, i]

        for i in range(0, self.uav_num - 1):
            for j in range(0, self.uav_num - 1):
                if (j + 1) == changed_id[i]:
                    new_formation[:, i] = position[:, j]
        return new_formation

    # def formation_switch_callback(self, msg):
    #     if not self.formation_config == msg.data:
    #         print("Follower"+str(self.id-1)+": Switch to Formation " + msg.data)
    #     self.formation_config = msg.data
    #     if self.formation_config == 'waiting':
    #         self.cmd_pub.publish(self.wait_cmd)
    #     else:
    #         self.formation_pattern = self.get_new_formation(self.changed_id, formation_dict[self.formation_config])
    #         self.L_matrix = self.get_L_matrix(self.formation_pattern)
    #         # Get the local leaders of this UAV based on the Laplacian matrix
    #         self.local_leader_ids = numpy.argwhere(self.L_matrix[self.id, :] == 1)
    #         self.local_leader_ids = self.local_leader_ids.reshape(self.local_leader_ids.shape[0])
    #         self.local_leader_count = len(self.local_leader_ids)
    #         self.weight = self.Kp / self.local_leader_count

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    def avoid_accel_callback(self, msg):
        self.avoid_accel = msg

    def loop(self):
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if not self.formation_config == 'waiting' and not self.L_matrix is None:
                input = Vector3(0, 0, 0)
                if (self.control_type == "vel"):
                    for local_leader_id in self.local_leader_ids:
                        if local_leader_id == 0: # The local leader is the global leader
                            print(self.changed_id[self.id - 1])
                            input.x += self.pose[local_leader_id].pose.position.x - (self.pose[self.id].pose.position.x -  self.formation_pattern[0, self.changed_id[self.id - 1]])
                            input.y += self.pose[local_leader_id].pose.position.y - (self.pose[self.id].pose.position.y -  self.formation_pattern[1, self.changed_id[self.id - 1]])
                            input.z += self.pose[local_leader_id].pose.position.z - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.changed_id[self.id - 1]])
                        else:
                            input.x += (self.pose[local_leader_id].pose.position.x - self.formation_pattern[0, local_leader_id - 1]) - (self.pose[self.id].pose.position.x -  self.formation_pattern[0, self.changed_id[self.id - 1]])
                            input.y += (self.pose[local_leader_id].pose.position.y - self.formation_pattern[1, local_leader_id - 1]) - (self.pose[self.id].pose.position.y -  self.formation_pattern[1, self.changed_id[self.id - 1]])
                            input.z += (self.pose[local_leader_id].pose.position.z - self.formation_pattern[2, local_leader_id]) - (self.pose[self.id].pose.position.z -  self.formation_pattern[2, self.changed_id[self.id - 1]])
                        # if (self.id == 1):
                            # print(self.pose[local_leader_id].pose.position.x - self.pose[self.id].pose.position.x)
                    self.cmd_vel_enu.linear.x = self.weight * input.x + self.vel_max * self.avoid_vel.x
                    self.cmd_vel_enu.linear.y = self.weight * input.y + self.vel_max * self.avoid_vel.y
                    self.cmd_vel_enu.linear.z = self.weight * input.z + self.vel_max * self.avoid_vel.z 
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
                                self.formation_pattern[0, self.changed_id[self.id - 1]] + self.gamma * (self.velocity[local_leader_id].twist.linear.x - self.velocity[self.id].twist.linear.x)
                            input.y += self.pose[local_leader_id].pose.position.y - self.pose[self.id].pose.position.y + \
                                self.formation_pattern[1, self.changed_id[self.id - 1]] + self.gamma * (self.velocity[local_leader_id].twist.linear.y - self.velocity[self.id].twist.linear.y)
                            input.z += self.pose[local_leader_id].pose.position.z - self.pose[self.id].pose.position.z + \
                                self.formation_pattern[2, self.changed_id[self.id - 1]] + self.gamma * (self.velocity[local_leader_id].twist.linear.z - self.velocity[self.id].twist.linear.z)
                        else:
                            input.x += self.pose[local_leader_id].pose.position.x - self.pose[self.id].pose.position.x + \
                                self.formation_pattern[0, self.changed_id[self.id - 1]] - self.formation_pattern[0, local_leader_id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.x - self.velocity[self.id].twist.linear.x)
                            input.y += self.pose[local_leader_id].pose.position.y - self.pose[self.id].pose.position.y + \
                                self.formation_pattern[1, self.changed_id[self.id - 1]] - self.formation_pattern[1, local_leader_id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.y - self.velocity[self.id].twist.linear.y)
                            input.z += self.pose[local_leader_id].pose.position.z - self.pose[self.id].pose.position.z + \
                                self.formation_pattern[2, self.changed_id[self.id - 1]] - self.formation_pattern[2, local_leader_id - 1] + self.gamma * (self.velocity[local_leader_id].twist.linear.z - self.velocity[self.id].twist.linear.z)
                        # if (self.id == 1):
                        #     print(self.pose[local_leader_id].pose.position.x - self.pose[self.id].pose.position.x)
                    self.cmd_accel_enu.linear.x = self.weight * input.x + self.accel_max * self.avoid_accel.x
                    self.cmd_accel_enu.linear.y = self.weight * input.y + self.accel_max * self.avoid_accel.y
                    self.cmd_accel_enu.linear.z = self.weight * input.z + self.accel_max * self.avoid_accel.z
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
