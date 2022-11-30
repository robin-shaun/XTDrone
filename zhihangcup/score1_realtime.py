#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Int16
from gazebo_msgs.msg import ModelStates
import time
from mavros_msgs.msg import State
import sys

def mavros_state_callback(msg):
    global armed
    armed = msg.armed

def gazebo_model_state_callback(msg):
    uav_id = msg.name.index('iris_0')
    target_id = msg.name.index('landing1')
    uav_pos = msg.pose[uav_id].position
    target_pos = msg.pose[target_id].position
    target_pos.y = target_pos.y + 3 #初始偏差
    time_usage = rospy.get_time() - start_time
    if(time_usage > 1200):
        print("Time out, score: 0")
    if(uav_pos.z < 0.15 and time_usage > 10 and not armed):
            distance = ((uav_pos.x - target_pos.x) ** 2 + (uav_pos.y - target_pos.y) ** 2) ** 0.5
            print("Arrived, distance is %.2f"%distance)
            if (time_usage < 600):
                score = 40
                if (distance <= 3):
                    score = score + 60 - 18 * distance                 
                else:
                    score = 0
            elif (time_usage < 1200):
                score = 40 - 3.6 * (time_usage/60 - 10)
                if (distance <= 3):
                    score = score + 60 - 18 * distance                 
                else:
                    score = 0
            else:
                score = 0

            print("Score: %.3f"%score)
            sys.exit(0)
        
if __name__ == "__main__":
    rospy.init_node('score_cal')
    time.sleep(1)
    armed = False
    mavros_sub = rospy.Subscriber("/iris_0/mavros/state", State, mavros_state_callback)
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
    target_finish = 0
    score = 0
    rate = rospy.Rate(20)
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        rate.sleep() 
