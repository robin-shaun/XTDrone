#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TwistStamped
from std_msgs.msg import String 
from pyquaternion import Quaternion
from formation_dict import formation_dict_6
import time
import math
import numpy 
import sys
import heapq
import copy
import Queue
from itertools import permutations

class Follower:

    def __init__(self, uav_id, uav_num):
        self.hover = "HOVER"
        self.offboard = "OFFBOARD"
        self.id = uav_id
        self.uav_num = uav_num
        self.f = 100
        self.local_pose = PoseStamped()
        self.local_pose_queue = Queue.Queue(self.f/10)
        for i in range(self.f/10):
            self.local_pose_queue.put(PoseStamped())
        self.local_velocity = TwistStamped()
        self.cmd_vel_enu = Twist()
        self.cmd_accel_enu = Vector3()
        self.avoid_accel = Vector3()
        self.following_switch = False
        self.arrive_print = True
        self.following_ids = []
        self.formation_config = 'waiting'
        self.following_count = 0
        self.Kp = 100 #100
        #self.kr = (4/int((self.uav_num-1)/2))**0.5
        self.kr = 1
        self.velxy_max = 2
        self.velz_max = 2
        self.following_local_pose = [PoseStamped() for i in range(self.uav_num)]
        self.following_local_pose_sub = [None]*self.uav_num
        self.following_local_velocity = [TwistStamped() for i in range(self.uav_num)]
        self.following_local_velocity_sub = [None]*self.uav_num
        self.arrive_count = 0
        self.local_pose_sub = rospy.Subscriber("/uav"+str(self.id)+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_velocity_sub = rospy.Subscriber("/uav"+str(self.id)+"/mavros/local_position/velocity_local", TwistStamped, self.local_velocity_callback)
        self.avoid_accel_sub = rospy.Subscriber("/xtdrone/uav"+str(self.id)+"/avoid_accel", Vector3, self.avoid_accel_callback)
        self.formation_switch_sub = rospy.Subscriber("/xtdrone/formation_switch",String, self.formation_switch_callback)
        self.vel_enu_pub = rospy.Publisher('/xtdrone/uav'+str(self.id)+'/cmd_vel_enu', Twist, queue_size=10)
        self.info_pub = rospy.Publisher('/xtdrone/uav'+str(self.id)+'/info', String, queue_size=10)
        self.cmd_pub = rospy.Publisher('/xtdrone/uav'+str(self.id)+'/cmd',String,queue_size=10)
        self.first_formation = True
        self.orig_formation = None
        self.new_formation = None

    def local_pose_callback(self, msg):
        self.local_pose = msg
        pose_comparison = self.local_pose_queue.get()
        self.local_pose_queue.put(self.local_pose)
        comparison = (self.local_pose.pose.position.x - pose_comparison.pose.position.x)**2+(self.local_pose.pose.position.y - pose_comparison.pose.position.y)**2+(self.local_pose.pose.position.z - pose_comparison.pose.position.z)**2
        if self.id == 6:
            print('comparison1:',comparison)
            print('comparison2:',float(self.velxy_max**2+self.velxy_max**2+self.velz_max**2)/1e3)
            print(self.arrive_count)
        if comparison < float(self.velxy_max**2+self.velxy_max**2+self.velz_max**2)/1e3:
            #if self.id == 2:
                #print('here')
            self.arrive_count += 1
        else:
            self.arrive_count = 0

    def local_velocity_callback(self, msg):
        self.local_velocity = msg


    def following_local_pose_callback(self, msg, id):
        self.following_local_pose[id] = msg 
       
    
    def following_local_velocity_callback(self, msg, id):
        self.following_local_velocity[id] = msg
        

    def cmd_vel_callback(self, msg):
        self.cmd_vel_enu = msg

    def formation_switch_callback(self, msg):
        if not self.formation_config == msg.data:
            self.following_switch = True
        self.formation_config = msg.data
        

    def avoid_accel_callback(self, msg):
        self.avoid_accel = msg

    def loop(self):
        rospy.init_node('follower'+str(self.id-1))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if self.arrive_count > 500 and self.arrive_print:
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
                    if self.formation_config=='waiting':
                        self.L_matrix = self.get_L_matrix(formation_dict_6[self.formation_config])
                    else:
                        if self.first_formation:
                            self.first_formation=False
                            self.orig_formation=formation_dict_6[self.formation_config]
                            self.L_matrix = self.get_L_matrix(formation_dict_6[self.formation_config])
                        else:
                            self.new_formation=self.get_new_formation(self.orig_formation,formation_dict_6[self.formation_config])
                            self.L_matrix = self.get_L_matrix(self.new_formation)
                            self.orig_formation=self.new_formation
                    #print(self.L_matrix)
                    #self.L_matrix = numpy.array([[0,0,0,0,0,0,0,0,0],[1,-1,0,0,0,0,0,0,0],[1,0,-1,0,0,0,0,0,0],[1,0,0,-1,0,0,0,0,0],[1,0,0,0,-1,0,0,0,0],[1,0,0,0,0,-1,0,0,0],[1,0,0,0,0,0,-1,0,0],[1,0,0,0,0,0,0,-1,0],[1,0,0,0,0,0,0,0,-1]])
                    self.following_ids = numpy.argwhere(self.L_matrix[self.id-1,:] == 1)
                    #if self.id == 2:
                        #print(self.following_ids)
                    self.following_count = 0
                    for i in range(self.uav_num):
                        if not self.following_local_pose_sub[i] == None:
                            self.following_local_pose_sub[i].unregister()
                        if not self.following_local_velocity_sub[i] == None:
                            self.following_local_velocity_sub[i].unregister()
                    for following_id in self.following_ids:
                        #print('here')
                        self.following_local_pose_sub[following_id[0]] = rospy.Subscriber("/uav"+str(following_id[0]+1)+"/mavros/local_position/pose", PoseStamped , self.following_local_pose_callback,following_id[0])
                        self.following_local_velocity_sub[following_id[0]] = rospy.Subscriber("/uav"+str(following_id[0]+1)+"/mavros/local_position/velocity_local", TwistStamped , self.following_local_velocity_callback,following_id[0])
                        self.following_count += 1


            self.cmd_accel_enu = Vector3(0, 0, 0)
            #self.cmd_vel_enu.linear = copy.deepcopy(self.avoid_accel)
            for following_id in self.following_ids:
                if self.following_local_pose[following_id[0]] == None and self.following_local_velocity[following_id[0]] == None:
                    print(following_id)     
                self.cmd_accel_enu.x += self.following_local_pose[following_id[0]].pose.position.x + self.kr * self.following_local_velocity[following_id[0]].twist.linear.x - self.local_pose.pose.position.x - self.kr * self.local_velocity.twist.linear.x + formation_dict_6[self.formation_config][0, self.id-2]
                self.cmd_accel_enu.y += self.following_local_pose[following_id[0]].pose.position.y + self.kr * self.following_local_velocity[following_id[0]].twist.linear.y - self.local_pose.pose.position.y - self.kr * self.local_velocity.twist.linear.y + formation_dict_6[self.formation_config][1, self.id-2]
                self.cmd_accel_enu.z += self.following_local_pose[following_id[0]].pose.position.z + self.kr * self.following_local_velocity[following_id[0]].twist.linear.z - self.local_pose.pose.position.z - self.kr * self.local_velocity.twist.linear.z + formation_dict_6[self.formation_config][2, self.id-2]
            
                if not following_id[0] == 0:
                    self.cmd_accel_enu.x -= formation_dict_6[self.formation_config][0, following_id[0]-1]
                    self.cmd_accel_enu.y -= formation_dict_6[self.formation_config][1, following_id[0]-1]
                    self.cmd_accel_enu.z -= formation_dict_6[self.formation_config][2, following_id[0]-1]
            if self.arrive_count <= 500:
                self.cmd_vel_enu.linear.x = self.local_velocity.twist.linear.x + self.Kp * (self.avoid_accel.x + self.cmd_accel_enu.x / self.f)
                self.cmd_vel_enu.linear.y = self.local_velocity.twist.linear.y + self.Kp * (self.avoid_accel.y + self.cmd_accel_enu.y / self.f)
                self.cmd_vel_enu.linear.z = self.local_velocity.twist.linear.z + self.Kp * (self.avoid_accel.z + self.cmd_accel_enu.z / self.f)
                if self.cmd_vel_enu.linear.x > self.velxy_max:
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
            else:
                self.cmd_pub.publish(self.hover)
            rate.sleep()

    def get_new_formation(self,orig_formation,change_formation):
        new_formation=numpy.zeros((3,self.uav_num-1))
        position=numpy.zeros((3,self.uav_num-1))

        num_array=range(0,self.uav_num-1)
        num_array_possi=list(permutations(num_array, self.uav_num-1))
        possi_num=len(num_array_possi)

        distance=[0 for i in range(self.uav_num-1)]
        distance_max=100   # maybe larger
        min_num=0

        for k in range(possi_num): 
            for i in range(self.uav_num-1):       
                distance[i]=numpy.linalg.norm(orig_formation[:,i]-change_formation[:,num_array_possi[k][i]])
            distance_mid_max=max(distance)
            if distance_mid_max<distance_max:
                distance_max=distance_mid_max
                min_num=k

        change_id=num_array_possi[min_num]
        change_id=[i + 1 for i in change_id] 
        #print (change_id)
        for i in range(0,self.uav_num-1):
            position[:,i]=orig_formation[:,i]

        for i in range(0,self.uav_num-1):
            for j in range(0,self.uav_num-1):
                if (j+1)==change_id[i]:
                    new_formation[:,i]=position[:,j]
        return new_formation

    #函数输入为相对leader的位置矩阵和无人机数量，输出为L矩阵
    def get_L_matrix(self,rel_posi):
	
        c_num=int((self.uav_num-1)/2)
        min_num_index_list = [0]*c_num
        
        comm=[[]for i in range (self.uav_num)]
        w=numpy.ones((self.uav_num,self.uav_num))*0 
        nodes_next=[]
        node_flag = [self.uav_num-1]
        node_mid_flag=[]

        rel_d=[0]*(self.uav_num-1)

        
        for i in range(0,self.uav_num-1):

            rel_d[i]=pow(rel_posi[0][i],2)+pow(rel_posi[1][i],2)+pow(rel_posi[2][i],2)

        c=numpy.copy(rel_d)
        c.sort()
        count=0

        for j in range(0,c_num):
            for i in range(0,self.uav_num-1):
                if rel_d[i]==c[j]:
	            if not i in node_mid_flag:
                        min_num_index_list[count]=i
	                node_mid_flag.append(i)
 	                count=count+1
                        if count==c_num:
		            break
            if count==c_num:
                break


        for j in range(0,c_num):
            nodes_next.append(min_num_index_list[j])
                        
            comm[self.uav_num-1].append(min_num_index_list[j])  

        size_=len(node_flag)

        while (nodes_next!=[]) and (size_<(self.uav_num-1)):

            next_node= nodes_next[0]
            nodes_next=nodes_next[1:]
            min_num_index_list = [0]*c_num
            node_mid_flag=[]
            rel_d=[0]*(self.uav_num-1)
            for i in range(0,self.uav_num-1):
                    
                if i==next_node or i in node_flag:
                        
                    rel_d[i]=2000   
                else:

                    rel_d[i]=pow((rel_posi[0][i]-rel_posi[0][next_node]),2)+pow((rel_posi[1][i]-rel_posi[1][next_node]),2)+pow((rel_posi[2][i]-rel_posi[2][next_node]),2)

            c=numpy.copy(rel_d)
    	    c.sort()
            count=0

            for j in range(0,c_num):
                for i in range(0,self.uav_num-1):
                    if rel_d[i]==c[j]:
	                if not i in node_mid_flag:
                            min_num_index_list[count]=i
	                    node_mid_flag.append(i)
			    count=count+1
                            if count==c_num:
		                break
                if count==c_num:
                    break
            node_flag.append(next_node)

            size_=len(node_flag)                    

            for j in range(0,c_num):

                if min_num_index_list[j] in node_flag:
                                
                    nodes_next=nodes_next

                else:
                    if min_num_index_list[j] in nodes_next:
                        nodes_next=nodes_next
                    else:
                        nodes_next.append(min_num_index_list[j])
                        
                    comm[next_node].append(min_num_index_list[j])

        for i in range (0,self.uav_num):
            for j in range(0,self.uav_num-1):

                if i==0:
                    if j in comm[self.uav_num-1]:
                        w[j+1][i]=1
                    else:
                        w[j+1][i]=0
                else:
                    if j in comm[i-1]:
                        w[j+1][i]=1
                    else:
                        w[j+1][i]=0
                        
        L=w  
        for i in range (0,self.uav_num):

            L[i][i]=-sum(w[i])

        return L 
'''
    def get_L_matrix(self, rel_posi):

        #假设无论多少UAV，都假设尽可能3层通信（叶子节点除外）
        c_num=int((self.uav_num-1)/3)
        
        comm=[[]for i in range (self.uav_num)]
        w=numpy.ones((self.uav_num,self.uav_num))*0 # 定义邻接矩阵
        nodes_next=[]
        node_flag = [self.uav_num-1]

        rel_d=[0]*(self.uav_num-1)
        # 规定每个无人机可以随机连接三台距离自己最近的无人机,且不能连接在flag中的无人机(即已经判断过连接点的无人机)。
        # 计算第一层通信（leader）：获得离自己最近的三台无人机编号
        
        for i in range(0,self.uav_num-1):

            rel_d[i]=pow(rel_posi[0][i],2)+pow(rel_posi[1][i],2)+pow(rel_posi[2][i],2)

        min_num_index_list = map(rel_d.index, heapq.nsmallest(c_num, rel_d))
        min_num_index_list=list(min_num_index_list)
        #leader 连接的无人机编号：
        comm[self.uav_num-1]=min_num_index_list

        nodes_next.extend(comm[self.uav_num-1])    

        size_=len(node_flag)

        while (nodes_next!=[]) and (size_<(self.uav_num-1)):

            next_node= nodes_next[0]
            nodes_next=nodes_next[1:]

            rel_d=[0]*(self.uav_num-1)
            for i in range(0,self.uav_num-1):
                    
                if i==next_node or i in node_flag:
                        
                    rel_d[i]=2000   #这个2000是根据相对位置和整个地图的大小决定的，要比最大可能相对距离大
                else:

                    rel_d[i]=pow((rel_posi[0][i]-rel_posi[0][next_node]),2)+pow((rel_posi[1][i]-rel_posi[1][next_node]),2)+pow((rel_posi[2][i]-rel_posi[2][next_node]),2)

            min_num_index_list =  map(rel_d.index, heapq.nsmallest(c_num, rel_d))
            min_num_index_list=list(min_num_index_list)
            node_flag.append(next_node)

            size_=len(node_flag)                    

            for j in range(0,c_num):

                if min_num_index_list[j] in node_flag:
                                
                    nodes_next=nodes_next

                else:
                    if min_num_index_list[j] in nodes_next:
                        nodes_next=nodes_next
                    else:
                        nodes_next.append(min_num_index_list[j])
                        
                    comm[next_node].append(min_num_index_list[j])
        # comm为每个uav连接其他uav的编号，其中数组的最后一行为leader   
        #print (comm)
        #leader是拉普拉斯矩阵的第一行，为0
        #第0号飞机（相对位置矩阵中的第一个位置）为第二行，以此类推

        for i in range (0,self.uav_num):
            for j in range(0,self.uav_num-1):

                if i==0:
                    if j in comm[self.uav_num-1]:
                        w[j+1][i]=1
                    else:
                        w[j+1][i]=0
                else:
                    if j in comm[i-1]:
                        w[j+1][i]=1
                    else:
                        w[j+1][i]=0
                        
        L=w  #定义拉普拉斯矩阵
        for i in range (0,self.uav_num):

            L[i][i]=-sum(w[i])
                    
        #print (L)               
        return L
'''

if __name__ == '__main__':
    follower = Follower(int(sys.argv[1]),int(sys.argv[2]))
    follower.loop()   