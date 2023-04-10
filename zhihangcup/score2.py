#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rosbag
import sys 
import math
import numpy as np
from scipy.spatial.transform import Rotation
import math
        
if __name__ == "__main__":
    armed = False
    rendezvous = False
    first_score = True
    first_record = True
    rendezvous_start_time = 0
    rendezvous_time = 1e6
    disarmed_time = 1e6
    score_sum = 0
    score_cnt = 1
    start_time = 0 
    score1 = 0 
    score2 = 0
    with rosbag.Bag('score2.bag') as bag:
        for topic, msg, time in bag.read_messages(topics=['/iris_0/mavros/state']):
            if(not armed and msg.armed):
                armed = True
                armed_time = time.to_sec()
            elif(armed and not msg.armed):
                armed = False
                disarmed_time = time.to_sec()
        for topic, msg, time in bag.read_messages(topics=['/rendezvous']):
            if(not rendezvous and msg.data):
                rendezvous = True
                rendezvous_time = time.to_sec()
        for topic, msg, time in bag.read_messages(topics=['/gazebo/relative_pose']):
            if(first_record):
                start_time = time.to_sec()
                first_record = False
            if(time.to_sec()>rendezvous_time and rendezvous):
                x0 = msg.position.x
                y0 = msg.position.y
                z0 = msg.position.z
                t = np.expand_dims([x0,y0,z0],axis=0)
                Rq = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
                rot = Rotation.from_quat(Rq)
                T = np.block([[rot.as_dcm(),t.transpose()],[np.zeros((1,3)),np.ones((1,1))]])
                distance = np.linalg.inv(T)[0,3]*100
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
                    print("Score1: %.4f"%score1)
                    first_score = False
                    rendezvous_start_time = time.to_sec()
                else:
                    score_sum = score_sum + (ring**2 + (10-np.min([10, abs(distance - 30)]))**2) / 4
                    score_cnt =  score_cnt + 1
                    if(time.to_sec()-rendezvous_start_time>10):
                        score2 = score_sum / score_cnt
                        print("Score2: %.4f"%score2) 
                        rendezvous = False 
            time_usage = time.to_sec() - start_time
            if(time_usage > 1200):
                print("Time out, score: 0")
                sys.exit(0) 
            if(time.to_sec() > disarmed_time):
                print("Mission finish, time usage: %.4f"%time_usage)
                score = score1 + score2
                print("Score: %.4f"%score) 
                sys.exit(0)    