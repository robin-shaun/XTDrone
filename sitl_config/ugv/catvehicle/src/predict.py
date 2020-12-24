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
import pickle

print("Tensforflow Version :{}".format(tf.__version__))

class predict():
    def __init__(self, ns):

        # Transformer for data scaling
        self.model_variables = None
        with open('/home/ivory/VersionControl/CopyCAT-reu2020/saved models/7-20-1-DeltaV/structure.pickle', mode="rb") as f:
            self.model_variables = pickle.load(f)


        print("===========")
        print("Model Data:")
        print(self.model_variables)
        
        self.history = self.model_variables[0]
        self.n_features = self.model_variables[1]

        self.deltaV_min = self.model_variables[3][0]
        self.deltaV_max = self.model_variables[3][1]
        self.dist_min =self.model_variables[5][0][0] # replace with the value from the file
        self.dist_max = self.model_variables[5][1][0] # replace with the value from the file
        self.accel_min =self.model_variables[5][0][1]  # replace with the value from the file
        self.accel_max = self.model_variables[5][1][1]  # replace with the value from the file
        self.vel_min = self.model_variables[5][0][2] # replace with the value from the file
        self.vel_max = self.model_variables[5][1][2] # replace with the value from the file

        print("deltaV_min: {}".format(self.deltaV_min))
        print("deltaV_max: {}".format(self.deltaV_max))
        print("dist_min: {}".format(self.dist_min))
        print("dist_max: {}".format(self.dist_max))
        print("accel_min: {}".format(self.accel_min))
        print("accel_max: {}".format(self.accel_max))
        print("vel_min: {}".format(self.vel_min))
        print("vel_max: {}".format(self.vel_max))
        print("===========")
        
        # self.dist_min = 0.3540591169968693 # replace with the value from the file
        # self.dist_max = 252.0000000000338 # replace with the value from the file
        # self.accel_min = -3.474197796605082  # replace with the value from the file
        # self.accel_max = 3.194018015245345  # replace with the value from the file
        # self.vel_min = -0.851840092450414  # replace with the value from the file
        # self.vel_max = 112.55704797888872  # replace with the value from the file

        self.dist_scaler = MinMaxScaler()
        self.accel_scaler = MinMaxScaler()
        self.vel_scaler = MinMaxScaler()

        self.dist_scaler.fit([[self.dist_min], [self.dist_max]])
        self.accel_scaler.fit([[self.accel_min], [self.accel_max]])
        self.vel_scaler.fit([[self.vel_min], [self.vel_max]])

        # history
        # self.history = 30 # replace the value form the file
        

        # Variables for the holding data
        self.dist_list = []
        self.vel_list = []
        self.accel_list = []
        
        self.last_velocity_time = None
        self.current_velocity_time = None

        self.last_distance_time = None
        self.current_distance_time = None

        self.new_vel_msg = False
        self.new_vel = 0.0

        self.new_distance_msg = False
        self.new_distance = 0.0

        self.ns = ns
        rospy.init_node("rnn_predict", anonymous=True)

        ## Publishers
        self.vel_pub = rospy.Publisher('cmd_vel'.format(ns), Twist, queue_size=1)

        # Load the saved model
        self.model = tf.keras.models.load_model("/home/ivory/VersionControl/CopyCAT-reu2020/saved models/7-20-1-DeltaV")

        ## Subscribers
        self.vel_sub = rospy.Subscriber('vel'.format(ns), Twist, self.vellcallback)
        self.distance_sub  = rospy.Subscriber('distanceEstimatorSteeringBased/dist'.format(ns), Float64, self.distance_calback)



    def vellcallback(self, data):
        """
        Velocity Call Back
        """

        # Retrieve Linear  X Component of the Velocity
        self.new_vel = data.linear.x

        if ((self.new_vel) < 0.0):
             self.new_vel  = 0.01

        # Add new velocity to the velocity list
        self.vel_list.append(self.new_vel)


        # Assign current velocity time to last velocity time before getting a new time
        self.last_velocity_time = self.current_velocity_time
        self.current_velocity_time = rospy.Time.now()

        if (self.last_velocity_time is not None) and (self.current_velocity_time is not None):
            duration = self.current_velocity_time - self.last_velocity_time
            deltaT = duration.to_sec()

            if deltaT == 0.0:
                self.new_vel_msg = False
                return
            
            # Calculate instantaneous acceleration
            accel = (self.vel_list[-1] - self.vel_list[-2])/deltaT
            self.accel_list.append(accel)
    
        # If a new velocity data point is received then set vel new to true
        self.new_vel_msg = True

    def distance_calback(self, data):
        """
        Distance Call Back
        """
        # Retrieve Linear  X Component of the Velocity
        # print("Current Distance is {}".format(data.data))
        self.new_distance = data.data

        # Add new velocity to the velocity list
        self.dist_list.append(self.new_distance)
        
        # Assign current velocity time to last velocity time before getting a new time
        self.last_distance_time = self.current_distance_time
        self.current_distance_time = rospy.Time.now()
    
        # If a new velocity data point is received then set vel new to true
        self.new_distance_msg = True

    @staticmethod
    def moving_avg(datalist, window_size = 10):
        pass
        
    def publish(self):
        """
        Publish Function 
        """
        # print("Inside publish function")
        if (self.new_distance_msg == True) or (self.new_vel_msg ==True):



            if (len(self.vel_list) <= self.history) or  (len(self.dist_list) <= self.history) or  (len(self.accel_list) <= self.history):
                return

            # scaled_vel_data = self.vel_scaler.transform([self.vel_list[-self.history:]])
            # scaled_dist_data = self.dist_scaler.transform([self.dist_list[-self.history:]])
            # scaled_accel_data = self.accel_scaler.transform([self.accel_list[-self.history:]])


            # TO DO
            # Use moving average smooth out acceleration history before using it make predictions

            # TO DO
            # How we create data vector for input to self.model.predict?
            scaled_data_vector = None
            scaled_data_vector = pd.DataFrame()

            # print("scaled_dist_data:")
            # print(scaled_dist_data[0].shape)
            # print(scaled_vel_data[0].shape)
            # print(scaled_accel_data[0].shape)

            # TO DO : reorder data columns as per specification
            # scaled_data_vector['dist'] = scaled_dist_data[0]
            # scaled_data_vector['accel'] = scaled_accel_data[0]
            # scaled_data_vector['speed'] = scaled_vel_data[0]

            scaled_data_vector['dist'] =  [(x -  self.dist_min)/(self.dist_max - self.dist_min) for x in self.dist_list[-self.history:]] 
            scaled_data_vector['accel'] = [(x -  self.accel_min)/(self.accel_max - self.accel_min) for x in self.accel_list[-self.history:]]  
            scaled_data_vector['speed'] = [(x -  self.vel_min)/(self.vel_max - self.vel_min) for x in self.vel_list[-self.history:]] 
            
            # print("data to be fed into model: ")
            # print(scaled_data_vector.tail(n=self.history))
            predicted_vel_diff = self.model.predict(scaled_data_vector.tail(n = self.history).to_numpy().reshape(1, self.history, 3))

            predicted_vel_diff_unscaled = predicted_vel_diff[0]*(self.deltaV_max  - self.deltaV_min) + self.deltaV_min

            print("Predicted vel diff is: {}".format(predicted_vel_diff_unscaled))

            new_vel =  self.vel_list[-1] + predicted_vel_diff_unscaled[0] + 0.1
            print("New Velocity is: {}".format(new_vel))
            
            new_vel_msg = Twist()
            new_vel_msg.linear.x = new_vel
            new_vel_msg.linear.y = 0.0
            new_vel_msg.linear.z = 0.0
            new_vel_msg.angular.x = 0.0
            new_vel_msg.angular.y = 0.0
            new_vel_msg.angular.z = 0.0

            self.vel_pub.publish(new_vel_msg)
            self.new_distance_msg = False
            self.new_vel_msg = False

def main(argv):
    ns = rospy.get_namespace() #Retrieve namespace this way appens '/' at the end as well,
    ns = ns[0:-1]
    node = predict(ns)

    rate = rospy.Rate(20) # 20 Hz publish rate

    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])


        