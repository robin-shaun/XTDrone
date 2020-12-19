#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Header, Float64
from geometry_msgs.msg  import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
import sys, getopt
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.animation as animation
import time
import sys
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import scipy.special as sp
from gazebo_msgs.srv import GetModelState
from sklearn.preprocessing import MinMaxScaler
import tensorflow as tf
import pandas as pd
import strym
from strym import strymread

print("Tensforflow Version: {}".format(tf.__version__))

#!/usr/bin/env python
# Initial Date: June 2020
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import roslaunch
import rospy, rosbag
import rospkg
import rostopic

import sys, math, time
import signal
import subprocess, shlex
from subprocess import call
import psutil
import numpy as np



""" This class is an utility class for creating and launching roslaunch, as well as terminating it """

'''
Summary of Class layout:
This class requires a ros package 'Sparkle'

Attributes:
    1. launchfile: The full path of the launch file
    2. theta: angular separation of two consecutive vehicle on the circle

    and all attributes of super class

Functions:
    1. __init__(circumference, n_vehicles): basically a constructor
'''
class launch:
    '''
    `launch`:A class facilitating roslaunch with runtime arguments and termination

    Parameters
    -------------

    launchfile: `string`
        Full path of the roslaunch file to be launch. Must be string. 

    kwargs
            variable keyword arguments passed as run-time argument for the launch file

    Attributes
    ------------
    launchfile: `string`
        Full path of the roslaunch file to be launch. Must be string. 

    runtime_args: `list`
        A list of run time arguments to be passed to launch file execution

    uuid: `string`
        Unique Identifier for the launch
    
    parent:  `roslaunch.parent.ROSLaunchParents`
        Length of car from bumper to bumper

    See Also
    ---------
    layout: superclass of `lane`

    '''
    def __init__(self, launchfile, **kwargs):
        self.launchfile = launchfile
        
        try:
            roslaunch.rlutil.resolve_launch_arguments([self.launchfile])    
        except roslaunch.RLException:
            print("Unable to find {}".format(self.launchfile))

        self.runtime_args = []
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        print(self.uuid)
        roslaunch.configure_logging(self.uuid)
        
        for key in kwargs.keys():
            self.runtime_args.append("{}:={}".format(key, kwargs[key]))

        if len(self.runtime_args) > 0:
            self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, [(self.launchfile, self.runtime_args)])
        else:
            self.parent = roslaunch.parent.ROSLaunchParent(self.uuid,[self.launchfile])
    def start(self):
        '''
        Calls `start()` function to execute roslaunch
        '''
        self.parent.start()
        time.sleep(5)
        if len(self.runtime_args) > 0:
            print("{} started with run-time arguments {}".format(self.launchfile, self.runtime_args))
        else:
            print("{} started.".format(self.launchfile))

    def shutdown(self):
        '''
        Calls `shutdown()` function to terminate the execution of the launch file
        '''
        self.parent.shutdown()
        print("{} Terminated.".format(self.launchfile))



class lead_drive():
    def __init__(self, ns, csvfile, dbcfile):

        self.ns = ns
        rospy.init_node("rnn_predict", anonymous=True)

        ## Publishers
        self.vel_pub = rospy.Publisher('cmd_vel'.format(ns), Twist, queue_size=1)

        self.r = strymread(csvfile=csvfile, dbcfile=dbcfile)

        # State information of lead from radar traces

        # self.long_dist = self.r.long_dist(np.arange(0, 16))
        # self.lat_dist = self.r.lat_dist(np.arange(0, 16))
        # self.rel_vel = self.r.rel_velocity(np.arange(0, 16))

        # # comine all tracks from radar
        # df_long_dist = pd.concat(long_dist)
        # df_lat_dist = pd.concat(lat_dist)
        # df_rel_vel = pd.concat(rel_vel)

        # # create a consolidated dataframe for vehicle state
        # df_long_dist['Long'] = df_long_dist['Message']
        # df_long_dist['Lat'] = df_lat_dist['Message']
        # df_long_dist['Relvel'] = df_rel_vel['Message']

        # df_long_dist.drop(columns=['Message'])
        
        # # Keep lead vehicle's information for which there is something in front of the car.
        # df_long_dist = df_long_dist[np.abs(df_long_dist['Lat']) < 1.0]

        # self.lead_state = df_long_dist    
        # # speed of RAV4 (called as ego vehicle)
        self.ego_speed = self.r.speed()
        self.ego_speed['Message'] = self.ego_speed['Message']*0.277778 # Convert to m/s
        # remove zero values from the beginning so that car moves immediately
        positive_vales = self.ego_speed[self.ego_speed['Message'] > 0.0]
        self.ego_speed = self.ego_speed[positive_vales.index[0]:]

        self.current_time = None
        self.next_time = None
        
    def publish(self):
        """
        Publish Function 
        """

        new_vel = self.ego_speed.iloc[0]['Message']
        self.current_time = self.ego_speed.iloc[0]['Time']

        if self.ego_speed.shape[0] == 1:
            self.next_time = -1 # -1 will signify that it is time to step publishing when last row has been read
            return

        self.next_time = self.ego_speed.iloc[1]['Time']

        # Remove the row just read from the dataframe
        self.ego_speed = self.ego_speed.iloc[1:]

        new_vel_msg = Twist()
        new_vel_msg.linear.x = new_vel
        new_vel_msg.linear.y = 0.0
        new_vel_msg.linear.z = 0.0
        new_vel_msg.angular.x = 0.0
        new_vel_msg.angular.y = 0.0
        new_vel_msg.angular.z = 0.0

        self.vel_pub.publish(new_vel_msg)

        

def main(argv):
    ns = rospy.get_namespace() #Retrieve namespace this way appends '/' at the end as well,
    ns = ns[0:-1]
  
    csvfile = argv[0]
    dbcfile = argv[1]
    node = lead_drive(ns, csvfile, dbcfile)

    #rate = rospy.Rate(20) # 20 Hz publish rate
    
    while not rospy.is_shutdown():
        node.publish()
        if node.next_time == -1:
            break
        deltaT = node.next_time - node.current_time
        time.sleep(deltaT)
        #rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
