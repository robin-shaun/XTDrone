#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Twist,Pose,PoseStamped,TwistStamped,Point
from std_msgs.msg import String
import sys 
Kp = 0.15
uav_num = int(sys.argv[1])
leader_id = 1
vision_pose = [None]*(uav_num+1)
relative_pose = [None]*(uav_num+1)
follower_vel_enu_pub = [None]*(uav_num+1)
relative_pose_pub = [None]*(uav_num+1)
follower_cmd_vel = [None]*(uav_num+1)
leader_cmd_vel = Twist()

for i in range(uav_num):
    vision_pose[i+1]=PoseStamped()
    relative_pose[i+1]=PoseStamped()
    follower_cmd_vel[i+1]=Twist()

#    uav_5 is the leader in the formation mission
formation=[None]*10


'''2x5 formation 0
fflff
fffff
'''
formation_temp = [None]*(uav_num+1)               
for i in range(uav_num):                           
    if i+1 <= (uav_num/2): 
        if (i+1)%2 == 1:                         
            formation_temp[i+1] = Point(0,i,0)
        else:
            formation_temp[i+1] = Point(0,-i-1,0)    
    else:                                         
        if (i+1)%2 == 1:                          
            formation_temp[i+1] = Point( -2, 1+i-(uav_num/2),0 )
        else:
            formation_temp[i+1] = Point( -2, (uav_num/2)-i ,0) 
formation[0] = formation_temp

''' 'L' formation  1
lffff
f
f
f
f   f
'''
formation_temp = [None]*(uav_num+1)         
for i in range(uav_num):                     
    if (i+1)%2 == 1:                       
        formation_temp[i+1] = Point(0,i,0)    
    elif i+1 != uav_num:                      
        formation_temp[i+1] = Point(-i-1,0,0)   
    else:                                   
        formation_temp[i+1] = Point(-4,4,0)
formation[1] = formation_temp               

''' '△' formation  2
       l
      f f
     f f f
    f f f f
'''
formation_temp = [None]*(uav_num+1)         
formation_temp[1]=Point(0,0,0)
formation_temp[2]=Point(-2,-2,0);formation_temp[3]=Point(-2,2,0)
formation_temp[4]=Point(-4,-4,0);formation_temp[5]=Point(-4,0,0);formation_temp[7]=Point(-4,4,0)
formation_temp[10]=Point(-6,-6,0);formation_temp[8]=Point(-6,-2,0);formation_temp[6]=Point(-6,2,0);formation_temp[9]=Point(-6,6,0)
formation[2] = formation_temp      

    


def leader_cmd_vel_callback(msg):
    global leader_cmd_vel
    leader_cmd_vel = msg

def calculate_relative_pose(uav_id):
    global relative_pose
    global vision_pose
    relative_pose[uav_id].pose.position.x = vision_pose[uav_id].pose.pose.position.x - vision_pose[leader_id].pose.pose.position.x
    relative_pose[uav_id].pose.position.y = vision_pose[uav_id].pose.pose.position.y - vision_pose[leader_id].pose.pose.position.y
    relative_pose[uav_id].pose.position.z = vision_pose[uav_id].pose.pose.position.z - vision_pose[leader_id].pose.pose.position.z
    
    
formation_id = 0

'''
formation.append( [[-1,-1],[0,-1],[1,-1],[-1,0],[0,0],[0,1]] )  #2x3 formation  
                                                              #  f l f 
                                                              #  f f f
formation.append(  [[-2,-2],[0,-2],[2,-2],[-1,-1],[0,0],[1,-1]] )#Trianglar formation
                                                              #     l
                                                              #    f  f
                                                              #   f  f  f
formation.append( [[0,-4],[0,-2],[0,-6],[-2,0],[0,0],[2,0]]  )  #'T' formation
                                                              #    f l f
                                                              #      f
                                                              #      f
                                                              #      f
formation_id = int(sys.argv[2])
'''

def leader_cmd_vel_callback(msg):
    global leader_cmd_vel
    leader_cmd_vel = msg

def calculate_relative_pose(uav_id):
    global relative_pose
    relative_pose[uav_id].pose.position.x = vision_pose[uav_id].pose.position.x - vision_pose[leader_id].pose.position.x
    relative_pose[uav_id].pose.position.y = vision_pose[uav_id].pose.position.y - vision_pose[leader_id].pose.position.y
    relative_pose[uav_id].pose.position.z = vision_pose[uav_id].pose.position.z - vision_pose[leader_id].pose.position.z

vision_pose_callback = [None]*(uav_num+1)

rospy.init_node('formation_control')

def vision_pose_callback(msg,id):
    global vision_pose
    vision_pose[id] = msg
    calculate_relative_pose(id)

def cmd_callback(msg):
    global formation_id
    if not msg.data == '': 
        formation_id = int(msg.data[-1])
        print("Switch to Formation"+str(formation_id))

for i in range(uav_num):
    uav_id = i+1
    rospy.Subscriber("/uav"+str(uav_id)+"/mavros/vision_pose/pose", PoseStamped , vision_pose_callback,uav_id)

leader_cmd_vel_sub = rospy.Subscriber("/xtdrone/leader/cmd_vel", Twist, leader_cmd_vel_callback)
formation_switch_sub = rospy.Subscriber("/xtdrone/leader_cmd",String, cmd_callback)
leader_vel_enu_pub =  rospy.Publisher('/xtdrone/uav'+str(leader_id)+'/cmd_vel_enu', Twist, queue_size=10)
for i in range(uav_num):
    uav_id = i+1
    if uav_id != leader_id:
        follower_vel_enu_pub[i+1] = rospy.Publisher(
         '/xtdrone/uav'+str(i+1)+'/cmd_vel_enu', Twist, queue_size=10)

rate = rospy.Rate(100)
while(1):
    for i in range(uav_num):
        uav_id = i+1
        if uav_id != leader_id:
            follower_cmd_vel[uav_id].linear.x = (leader_cmd_vel.linear.x+Kp*(formation[formation_id][uav_id].x- relative_pose[uav_id].pose.position.x) ) 
            follower_cmd_vel[uav_id].linear.y = (leader_cmd_vel.linear.y+Kp*(formation[formation_id][uav_id].y- relative_pose[uav_id].pose.position.y) )
            follower_cmd_vel[uav_id].linear.z = leader_cmd_vel.linear.z
            follower_cmd_vel[uav_id].angular.x = 0.0; follower_cmd_vel[uav_id].angular.y = 0.0;  follower_cmd_vel[uav_id].angular.z = 0.0
            follower_vel_enu_pub[uav_id].publish(follower_cmd_vel[uav_id])
    leader_vel_enu_pub.publish(leader_cmd_vel)
    rate.sleep()
