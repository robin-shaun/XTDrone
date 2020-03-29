import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   
import matplotlib.animation as animation              
import numpy as np 
import rospy
from geometry_msgs.msg import Twist,Pose,PoseStamped,TwistStamped,Vector3
from gazebo_msgs.srv import GetModelState
import sys
from std_msgs.msg import String 
from copy import deepcopy


uav_num = int(sys.argv[1])

step_time=0.01
Kp=100

pose_puber=[None]*uav_num
vel_puber=[None]*uav_num

plot_x=[0]*uav_num
plot_y=[0]*uav_num
plot_z=[0]*uav_num
local_vel = [TwistStamped() for i in range (uav_num)]
cmd_vel=[Twist() for i in range (uav_num)]
arrive_count=0

for i in range(uav_num):
    uav_id = i + 1
    plot_x[i]= i//3
    plot_y[i]= i%3
    pose_puber[i]=rospy.Publisher('/uav'+str(uav_id)+'/mavros/local_position/pose', PoseStamped, queue_size=10)
    vel_puber[i]=rospy.Publisher('/uav'+str(uav_id)+'/mavros/local_position/velocity_local', TwistStamped, queue_size=10)



def cmd_accel_callback(msg,id):
    cmd_vel[id].linear.x = cmd_vel[id].linear.x + Kp * msg.x * step_time
    cmd_vel[id].linear.y = cmd_vel[id].linear.y + Kp * msg.y * step_time
    cmd_vel[id].linear.z = cmd_vel[id].linear.z + Kp * msg.z * step_time

    plot_x[id]+=step_time*cmd_vel[id].linear.x
    plot_y[id]+=step_time*cmd_vel[id].linear.y
    plot_z[id]+=step_time*cmd_vel[id].linear.z
    if id == 0:
        print(id)
    local_vel[id].twist=deepcopy(cmd_vel[id])


rospy.init_node('simple_3d_simulator')
rate = rospy.Rate(1/step_time)

for i in range(uav_num):
    uav_id = i + 1
    rospy.Subscriber('/xtdrone/uav'+str(uav_id)+'/cmd_accel_enu',Vector3, cmd_accel_callback,i) 

while not rospy.is_shutdown():

    for i in range(uav_num):
        local_pose=PoseStamped()
        local_pose.pose.position.x=plot_x[i]
        local_pose.pose.position.y=plot_y[i]
        local_pose.pose.position.z=plot_z[i]    
        pose_puber[i].publish(local_pose)
        vel_puber[i].publish(local_vel[i])