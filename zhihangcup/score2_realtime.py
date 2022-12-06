#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Pose
import time
from mavros_msgs.msg import State
import os
import numpy as np
from scipy.spatial.transform import Rotation
import math

def mavros_state_callback(msg):
    global armed
    armed = msg.armed

def rendezvous_callback(msg):
    global rendezvous
    rendezvous = msg.data

def distance_callback(msg):
    global distance
    distance = msg.data

def relative_pose_callback(msg):
    global rendezvous, score_sum, score_cnt, start_time, first_score, rendezvous_start_time, score1
    if(rendezvous):
        x0 = msg.position.x
        y0 = msg.position.y
        z0 = msg.position.z
        Rq = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        rot = Rotation.from_quat(Rq)
        euler = rot.as_euler('zyx', degrees=False)
        y = -x0 * math.tan(euler[0]) + y0
        z = -x0 * math.tan(euler[1]) / math.cos(euler[0]) + z0
        r = (y**2+z**2)**0.5
        ring = 10 - math.floor(r / 0.0225)
        if(ring < 0):
            ring = 0
        if(first_score):
            score1 = (ring**2 + (10-np.min([10, abs(distance - 30)]))**2) / 4
            score_sum = score1
            score_cnt =  score_cnt + 1
            print("Score1: %.3f"%score1)
            first_score = False
            rendezvous_start_time = rospy.get_time()
        else:
            score_sum = score_sum + (ring**2 + (10-np.min([10, abs(distance - 30)]))**2) / 4
            print("Score sum: %.3f"%score_sum)
            score_cnt =  score_cnt + 1
            print("Score cnt: %d"%score_cnt)
            if(rospy.get_time()-rendezvous_start_time>10):
                score2 = score_sum / score_cnt
                print("Score2: %.3f"%score2) 
                score = score1 + score2
                print("Score: %.3f"%score)
                rendezvous = False 
    time_usage = rospy.get_time() - start_time
    if(time_usage > 1200):
        print("Time out, score: 0")
        os._exit(0) 
    if(time_usage > 30 and not armed):
        print("Mission finish, time usage: %.3f"%time_usage) 
        os._exit(0)    

if __name__ == "__main__":
    rospy.init_node('score_cal')
    time.sleep(1)
    armed = False
    rendezvous = False
    first_score = True
    rendezvous_start_time = rospy.get_time()
    mavros_sub = rospy.Subscriber("/iris_0/mavros/state", State, mavros_state_callback,queue_size=1)
    rendezvous_sub = rospy.Subscriber("/rendezvous", Bool, rendezvous_callback,queue_size=1)
    distance_sub = rospy.Subscriber("/gazebo/distance", Float32, distance_callback,queue_size=1)
    relative_pose_sub = rospy.Subscriber("/gazebo/relative_pose", Pose, relative_pose_callback,queue_size=1)
    distance = 100.0
    score_sum = 0.0
    score_cnt = 0
    score1 = 0
    rate = rospy.Rate(20)
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        rate.sleep()  
