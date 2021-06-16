#!/usr/bin/python
# -*- coding: UTF-8 -*-
### This code is about the distributed formation control of consensus protocol with a certain
### Laplacian matrix and the formation transformation based on a task allocation algorithm——
### KM for the shorest distances of all the UAVs to achieve the new pattern.
### For more information of these two algorithms, please see the latest paper on https://arxiv.org/abs/2005.01125

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TwistStamped
from std_msgs.msg import String 
import sys

# formation patterns
#if sys.argv[3] == '6':
#    from formation_dict import formation_dict_6 as formation_dict
#elif sys.argv[3] == '9':
#    from formation_dict import formation_dict_9 as formation_dict
#elif sys.argv[3] == '18':
#    from formation_dict import formation_dict_18 as formation_dict
if sys.argv[3] == '21':
    from my_formation_dict import formation_dict_my as formation_dict
elif sys.argv[3] == '34':
    from my_formation_dict import formation_dict_my as formation_dict

import numpy 
import Queue


class Follower:

    def __init__(self, uav_type, uav_id, uav_num):
        self.hover = "HOVER"
        self.offboard = "OFFBOARD"
        self.uav_type = uav_type
        self.id = uav_id
        self.uav_num = uav_num
        self.f = 100 # control/communication rate
        self.local_pose = PoseStamped()
        self.local_pose_queue = Queue.Queue(self.f/10)
        for i in range(self.f/10):
            self.local_pose_queue.put(PoseStamped())
        self.local_velocity = TwistStamped()
        self.cmd_vel_enu = Twist()
        self.avoid_vel = Vector3()
        self.following_switch = False # determine whether the formation pattern is required to be changed
        self.arrive_print = True # determine whether the target position has been reached
        self.following_ids = [] # followers of this uav
        self.formation_config = 'waiting'
        self.following_count = 0 # the number of followers of this uav
        self.Kp = 1 
        self.velxy_max = 1 #0.8
        self.velz_max = 1
        self.following_local_pose = [PoseStamped() for i in range(self.uav_num)] # local position of other uavs, and only the position of followers of this uav is not zero
        self.following_local_pose_sub = [None]*self.uav_num
        self.arrive_count = 0
        ###############起飞位置偏置
        self.bias = [ [3, 3, 0],[6, 3, 0],[9, 3, 0],[12, 3, 0],[15, 3, 0],[18, 3, 0], [0, 6, 0],[3, 6, 0],[6, 6, 0],[9, 6, 0],[12, 6, 0],[15, 6, 0],[18, 6, 0], [0, 9, 0],[3, 9, 0],[6, 9, 0],[9, 9, 0],[12, 9, 0],[15, 9, 0],[18, 9, 0] ]
        self.local_pose_sub = rospy.Subscriber(self.uav_type+'_'+str(self.id)+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+self.uav_type+'_'+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback)
        self.formation_switch_sub = rospy.Subscriber("/xtdrone/formation_switch",String, self.formation_switch_callback)
        self.vel_enu_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd_vel_enu', Twist, queue_size=10)
        self.info_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/info', String, queue_size=10)
        self.cmd_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd',String,queue_size=10)
        self.first_formation = True
        self.orig_formation = None
        self.new_formation = None

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_pose.pose.position.x += self.bias[self.id-1][0]###########起飞位置偏置
        self.local_pose.pose.position.y += self.bias[self.id-1][1]

        pose_comparison = self.local_pose_queue.get()
        self.local_pose_queue.put(self.local_pose)
        comparison = (self.local_pose.pose.position.x - pose_comparison.pose.position.x)**2+(self.local_pose.pose.position.y - pose_comparison.pose.position.y)**2+(self.local_pose.pose.position.z - pose_comparison.pose.position.z)**2
        if comparison < float(self.velxy_max**2+self.velxy_max**2+self.velz_max**2)/1e5: # if the target position is reached, arrive_count ++1
            self.arrive_count += 1
        else:
            self.arrive_count = 0

    def following_local_pose_callback(self, msg, id):
        self.following_local_pose[id] = msg 
        self.following_local_pose[id].pose.position.x += self.bias[id][0]#############起飞位置偏置
        self.following_local_pose[id].pose.position.y += self.bias[id][1]

    # the order of changing the formation pattern
    def formation_switch_callback(self, msg):
        if not self.formation_config == msg.data:
            self.following_switch = True
        self.formation_config = msg.data   

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    def loop(self):
        rospy.init_node('follower'+str(self.id-1))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if self.arrive_count > 2000 and self.arrive_print:
                print("Follower"+str(self.id-1)+":Arrived")
                self.arrive_print = False
            if self.following_switch:
                self.following_switch = False
                self.arrive_print = True
                self.arrive_count = 0
                for i in range(self.f/10):
                    self.cmd_pub.publish(self.offboard)
                    self.info_pub.publish("Received")
                    print("Follower"+str(self.id-1)+": Switch to Formation "+self.formation_config)
                    # The Laplacian matrix is invarible in this code, and you can change it if necessary.
                    if self.formation_config=='waiting': 
                        self.L_matrix = self.get_L_central_matrix()
                    else:
                        # Change from the original pattern to the first pattern without KM.
                        if self.first_formation: 
                            self.first_formation=False
                            self.orig_formation=formation_dict[self.formation_config]
                            self.L_matrix = self.get_L_central_matrix()
                        else:
                            self.adj_matrix = self.build_graph(self.orig_formation,formation_dict[self.formation_config])
                            # These variables are determined for KM algorithm, see examples of KM algorithm on Github.
                            self.label_left = numpy.max(self.adj_matrix, axis=1)  # init label for the left 
                            self.label_right = numpy.array([0]*(self.uav_num-1)) # init label for the right set
                            self.match_right = numpy.array([-1] *(self.uav_num-1))
                            self.visit_left = numpy.array([0]*(self.uav_num-1))
                            self.visit_right = numpy.array([0]*(self.uav_num-1))
                            self.slack_right = numpy.array([100]*(self.uav_num-1)) 

                            self.change_id = self.KM()
                            # Get a new formation pattern of UAVs based on KM.
                            self.new_formation=self.get_new_formation(self.change_id,formation_dict[self.formation_config])
                            self.L_matrix = self.get_L_central_matrix()
                            self.orig_formation=self.new_formation
                    if self.id == 3:
                        print(self.L_matrix)
                    # Get the followers of this uav based on the Laplacian matrix, and update the position of the followers.
                    self.following_ids = numpy.argwhere(self.L_matrix[self.id,:] == 1)
                    self.following_count = 0
                    for i in range(self.uav_num):
                        if not self.following_local_pose_sub[i] == None:
                            self.following_local_pose_sub[i].unregister()
                    for following_id in self.following_ids:
                        self.following_local_pose_sub[following_id[0]] = rospy.Subscriber(self.uav_type+'_'+str(following_id[0])+"/mavros/local_position/pose", PoseStamped , self.following_local_pose_callback,following_id[0])
                        self.following_count += 1
                      
            self.cmd_vel_enu.linear = Vector3(0, 0, 0)
            # Code of the consensus protocol, see details on the paper.
            for following_id in self.following_ids:
                #print(self.id,"following: ", self.following_ids)
                self.cmd_vel_enu.linear.x += self.following_local_pose[following_id[0]].pose.position.x - self.local_pose.pose.position.x + self.new_formation[0, self.id-1]
                self.cmd_vel_enu.linear.y += self.following_local_pose[following_id[0]].pose.position.y - self.local_pose.pose.position.y + self.new_formation[1, self.id-1]
                self.cmd_vel_enu.linear.z += self.following_local_pose[following_id[0]].pose.position.z - self.local_pose.pose.position.z + self.new_formation[2, self.id-1]
                if not following_id[0] == 0:
                    self.cmd_vel_enu.linear.x -= self.new_formation[0, following_id[0]-1]
                    self.cmd_vel_enu.linear.y -= self.new_formation[1, following_id[0]-1]
                    self.cmd_vel_enu.linear.z -= self.new_formation[2, following_id[0]-1]

            self.cmd_vel_enu.linear.x = self.Kp * self.cmd_vel_enu.linear.x + self.avoid_vel.x
            self.cmd_vel_enu.linear.y = self.Kp * self.cmd_vel_enu.linear.y + self.avoid_vel.y
            self.cmd_vel_enu.linear.z = self.Kp * self.cmd_vel_enu.linear.z + self.avoid_vel.z

            if self.cmd_vel_enu.linear.x > self.velxy_max: #越界限制
                self.cmd_vel_enu.linear.x = self.velxy_max
            elif self.cmd_vel_enu.linear.x < - self.velxy_max:
                self.cmd_vel_enu.linear.x = - self.velxy_max
            if self.cmd_vel_enu.linear.y > self.velxy_max:
                self.cmd_vel_enu.linear.y = self.velxy_max
            elif self.cmd_vel_enu.linear.y < - self.velxy_max:
                self.cmd_vel_enu.linear.y = - self.velxy_max
            if self.cmd_vel_enu.linear.z > self.velz_max:
                self.cmd_vel_enu.linear.z = self.velz_max
            elif self.cmd_vel_enu.linear.z < - self.velz_max:
                self.cmd_vel_enu.linear.z = - self.velz_max

            if not self.formation_config == 'waiting':
                self.vel_enu_pub.publish(self.cmd_vel_enu)
            if (self.cmd_vel_enu.linear.x)**2+(self.cmd_vel_enu.linear.y)**2+(self.cmd_vel_enu.linear.z)**2<0.2: #到达目标位置
                self.arrive_count += 1
            else:
                self.arrive_count = 0
            rate.sleep()

    # 'build_graph',  'find_path' and 'KM' functions are all determined for KM algorithm.
    # A graph of UAVs is established based on distances between them in 'build_graph' function.
    def build_graph(self,orig_formation,change_formation):
        distance=[[0 for i in range(self.uav_num-1)]for j in range(self.uav_num-1)] #二维0矩阵，行列都为跟随者个数
        for i in range(self.uav_num-1):
            for j in range(self.uav_num-1):
                distance[i][j]=numpy.linalg.norm(orig_formation[:,i]-change_formation[:,j])
                distance[i][j]=int(50-distance[i][j])
        return distance

    # Determine whether a path has been found.
    def find_path(self,i):
        self.visit_left[i] = True
        for j, match_weight in enumerate(self.adj_matrix[i],start=0): 
            if self.visit_right[j]: 
                continue  
            gap = self.label_left[i] + self.label_right[j] - match_weight 
            if gap == 0 : 
                self.visit_right[j] = True   
                if self.match_right[j]==-1 or self.find_path(self.match_right[j]): 
                    self.match_right[j] = i     
                    return True            
            else: 
                self.slack_right[j]=min(gap,self.slack_right[j])          
        return False 

    # Main body of KM algorithm.
    def KM(self):  
        for i in range(self.uav_num-1):   
            self.slack_right = numpy.array([100]*(self.uav_num-1))      
            while True:        
                self.visit_left = numpy.array([0]*(self.uav_num-1))                
                self.visit_right = numpy.array([0]*(self.uav_num-1))               
                if self.find_path(i):    
                    break       
                d = numpy.inf
                for j, slack in enumerate(self.slack_right):         
                    if not self.visit_right[j] :           
                        d = min(d,slack)  
                for k in range(self.uav_num-1):          
                    if self.visit_left[k]: 
                        self.label_left[k] -= d                 
                    if self.visit_right[k]: 
                        self.label_right[k] += d   
                    else:
                        self.slack_right[k] -=d 
        return self.match_right
    
    # The formation patterns designed in the formation dictionaries are random (the old ones), 
    # and a new formation pattern based on the distances of UAVs of the current pattern is designed as follows.
    # Note that only the desired position of each UAV has changed, while the form of the new pattern is the same as the one in the dictionary.
    def get_new_formation(self,change_id,change_formation):
        new_formation=numpy.zeros((3,self.uav_num-1))
        position=numpy.zeros((3,self.uav_num-1))
        change_id=[i + 1 for i in change_id] 
        for i in range(0,self.uav_num-1):
            position[:,i]=change_formation[:,i]

        for i in range(0,self.uav_num-1):
            for j in range(0,self.uav_num-1):
                if (j+1)==change_id[i]:
                    new_formation[:,i]=position[:,j]
        return new_formation

    # Laplacian matrix 
    def get_L_central_matrix(self):
        
        L=numpy.zeros((self.uav_num,self.uav_num))
        for i in range(1,self.uav_num):
            L[i][0]=1
            L[i][i]=-1
        
        #L=numpy.array([[-0. ,0. , 0.,  0.,  0.,  0.],[ 1. ,-1. , 0. , 0. , 0. , 0.],[ 0.,  1. ,-1. , 0. , 0. , 0.],[ 0.,  0.,  1. ,-1. , 0. , 0.],[ 0. , 0. , 0. , 1. ,-1. , 0.],[ 0. , 0. , 0. , 0. , 1. ,-1.]])
        return L

if __name__ == '__main__':
    follower = Follower(sys.argv[1],int(sys.argv[2]),int(sys.argv[3]))
    follower.loop()   
