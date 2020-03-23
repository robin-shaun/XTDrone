#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Twist,Pose,PoseStamped,TwistStamped,Point
from std_msgs.msg import String
import sys 
from pyquaternion import Quaternion
import time
KP_xy = 1.5
KP_yaw = 2.8
KP_z = 1
max_vel_xy = 1
max_vel_z = 0.6
uav_num = int(sys.argv[1])
leader_id = 1
local_pose = [None]*(uav_num+1)
relative_pose = [None]*(uav_num+1)
follower_vel_enu_pub = [None]*(uav_num+1)
relative_pose_pub = [None]*(uav_num+1)
follower_cmd_vel = [None]*(uav_num+1)
leader_cmd_vel = Twist()
avoid_vel_z = [0]*(uav_num+1)
avoid_pos_z = 0.3
leader_height = 0
hover = True
avoid = False

for i in range(uav_num):
    local_pose[i+1]=PoseStamped()
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
    global leader_cmd_vel, hover
    leader_cmd_vel = msg
    #print(hover)
    if msg.linear.z == 0:
        hover = True
    else:
        hover = False

def calculate_relative_pose(uav_id):
    global relative_pose
    global local_pose
    relative_pose[uav_id].pose.position.x = local_pose[uav_id].pose.pose.position.x - local_pose[leader_id].pose.pose.position.x
    relative_pose[uav_id].pose.position.y = local_pose[uav_id].pose.pose.position.y - local_pose[leader_id].pose.pose.position.y
    relative_pose[uav_id].pose.position.z = local_pose[uav_id].pose.pose.position.z - local_pose[leader_id].pose.pose.position.z

def delta_vel(target_pos, current_pos, KP, vel_max):
    delta_vel = KP*(target_pos-current_pos)
    if delta_vel > vel_max:
        delta_vel = vel_max
    return delta_vel   
    
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


def calculate_relative_pose(uav_id):
    global relative_pose
    relative_pose[uav_id].pose.position.x = local_pose[uav_id].pose.position.x - local_pose[leader_id].pose.position.x
    relative_pose[uav_id].pose.position.y = local_pose[uav_id].pose.position.y - local_pose[leader_id].pose.position.y
    relative_pose[uav_id].pose.position.z = local_pose[uav_id].pose.position.z - local_pose[leader_id].pose.position.z



rospy.init_node('formation_control')

def local_pose_callback(msg,id):
    global local_pose
    local_pose[id] = msg
    calculate_relative_pose(id)

def cmd_callback(msg):
    global formation_id
    if not msg.data == '': 
        formation_id = int(msg.data[-1])
        print("Switch to Formation"+str(formation_id))

for i in range(uav_num):
    uav_id = i+1
    rospy.Subscriber("/uav"+str(uav_id)+"/mavros/local_position/pose", PoseStamped , local_pose_callback,uav_id)

leader_cmd_vel_sub = rospy.Subscriber("/xtdrone/leader/cmd_vel", Twist, leader_cmd_vel_callback)
formation_switch_sub = rospy.Subscriber("/xtdrone/leader_cmd",String, cmd_callback)
leader_vel_enu_pub =  rospy.Publisher('/xtdrone/uav'+str(leader_id)+'/cmd_vel_enu', Twist, queue_size=10)
for i in range(uav_num):
    uav_id = i+1
    if uav_id != leader_id:
        follower_vel_enu_pub[i+1] = rospy.Publisher(
         '/xtdrone/uav'+str(i+1)+'/cmd_vel_enu', Twist, queue_size=10)
for i in range(10):
    leader_height = local_pose[leader_id].pose.position.z 
    time.sleep(0.1)
rate = rospy.Rate(100)

while(1):

    leader_orientation = Quaternion(local_pose[leader_id].pose.orientation.w,local_pose[leader_id].pose.orientation.x,local_pose[leader_id].pose.orientation.y,local_pose[leader_id].pose.orientation.z)
    leader_yaw = leader_orientation.yaw_pitch_roll[0]
     # Avoid collision with other drones 
    for i in range(uav_num): 
        uav_id = i+1      
        for j in range(1,uav_num-i):            
            if pow(local_pose[uav_id].pose.position.x-local_pose[uav_id+j].pose.position.x,2)\
                +pow(local_pose[uav_id].pose.position.y-local_pose[uav_id+j].pose.position.y,2)\
                 +pow(local_pose[uav_id].pose.position.z-local_pose[uav_id+j].pose.position.z,2)  < 1:
                avoid = True
                avoid_vel_z[uav_id] = KP_z*avoid_pos_z
                avoid_vel_z[uav_id+j] = -KP_z*avoid_pos_z
            else:
                avoid_vel_z[uav_id] = 0
                avoid_vel_z[uav_id+j] = 0
    for i in range(uav_num):
        uav_id = i+1
        if uav_id != leader_id:
            follower_cmd_vel[uav_id].linear.x = leader_cmd_vel.linear.x+delta_vel(formation[formation_id][uav_id].x,relative_pose[uav_id].pose.position.x,KP_xy, max_vel_xy) 
            follower_cmd_vel[uav_id].linear.y = leader_cmd_vel.linear.y+delta_vel(formation[formation_id][uav_id].y,relative_pose[uav_id].pose.position.y, KP_xy, max_vel_xy) 
            follower_cmd_vel[uav_id].linear.z = leader_cmd_vel.linear.z + delta_vel(leader_height,local_pose[uav_id].pose.position.z, KP_z, max_vel_z) + avoid_vel_z[uav_id] - avoid_vel_z[leader_id]

            orientation = Quaternion(local_pose[uav_id].pose.orientation.w,local_pose[uav_id].pose.orientation.x,local_pose[uav_id].pose.orientation.y,local_pose[uav_id].pose.orientation.z)
            yaw = orientation.yaw_pitch_roll[0]
            follower_cmd_vel[uav_id].angular.x = 0.0; follower_cmd_vel[uav_id].angular.y = 0.0; follower_cmd_vel[uav_id].angular.z = KP_yaw*(leader_yaw - yaw)
            
            follower_vel_enu_pub[uav_id].publish(follower_cmd_vel[uav_id])

    if hover: 
        leader_cmd_vel.linear.z = delta_vel(leader_height,local_pose[leader_id].pose.position.z, KP_z, max_vel_z) + avoid_vel_z[leader_id]
    else:
        leader_cmd_vel.linear.z = leader_cmd_vel.linear.z + avoid_vel_z[leader_id]
        if not avoid:
            leader_height = local_pose[leader_id].pose.position.z
    leader_vel_enu_pub.publish(leader_cmd_vel)

    print(leader_height)

    rate.sleep()
