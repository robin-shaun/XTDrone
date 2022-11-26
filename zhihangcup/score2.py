#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Int16, Bool
from gazebo_msgs.srv import GetModelState, GetLinkState
from sensor_msgs.msg import Image
import time
from mavros_msgs.msg import State
import sys
from cv_bridge import CvBridge
bridge = CvBridge()
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import math
img_width = 1280
img_height = 720

def mavros_state_callback(msg):
    global armed
    armed = msg.armed

def rendezvous_callback(msg):
    global rendezvous
    rendezvous = msg.data

if __name__ == "__main__":
    rospy.init_node('score_cal')
    time.sleep(1)
    armed = False
    rendezvous = False
    first_score = True
    mavros_sub = rospy.Subscriber("/iris_0/mavros/state", State, mavros_state_callback)
    rendezvous_sub = rospy.Subscriber("/rendezvous", Bool, rendezvous_callback)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState) 
    get_link_state = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
    target_finish = 0
    score1 = 0.0
    score2 = 0.0
    score_sum = 0.0
    total_score = 0.0
    score_cnt = 0
    rate = rospy.Rate(20)
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        if(rendezvous):
            try:
                response1 = get_link_state('target_green::link','iris_0::realsense_camera::link')
                response2 = get_link_state('iris_0::realsense_camera::link', 'target_green::link')
            except:
                print("Gazebo model state service call failed")
            distance = abs(response1.link_state.pose.position.x) * 100
            x0 = response2.link_state.pose.position.x
            y0 = response2.link_state.pose.position.y
            z0 = response2.link_state.pose.position.z
            Rq = [response2.link_state.pose.orientation.x,response2.link_state.pose.orientation.y,response2.link_state.pose.orientation.z,response2.link_state.pose.orientation.w]
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
                start_time = rospy.get_time()
            else:
                score_sum = score_sum + (ring**2 + (10-np.min([10, abs(distance - 30)]))**2) / 4
                score_cnt =  score_cnt + 1
                if(rospy.get_time()-start_time>10):
                    score2 = score_sum / score_cnt
                    print("Score2: %.3f"%score2) 
                    score = score1 + score2
                    print("Score: %.3f"%score)
                    rendezvous = False 
        time_usage = rospy.get_time() - start_time
        if(time_usage > 1200):
            print("Time out, score: 0")
            sys.exit(0) 
        if(time_usage > 30 and not armed):
            print("Mission finish, time usage: %.3f"%time_usage) 
            sys.exit(0)      
