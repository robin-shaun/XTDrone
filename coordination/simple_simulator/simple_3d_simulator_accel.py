import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   
import matplotlib.animation as animation              
import numpy as np 
import rospy
from geometry_msgs.msg import Twist,Pose,PoseStamped,TwistStamped
from gazebo_msgs.srv import GetModelState
import sys

uav_type = sys.argv[1]
uav_num = int(sys.argv[2])

step_time=0.005

pose_puber=[None]*uav_num
vel_puber=[None]*uav_num

plot_x=[0]*(uav_num)
plot_y=[0]*(uav_num)
plot_z=[0]*(uav_num)
local_vel = [TwistStamped() for i in range (uav_num)]

for i in range(uav_num):
    plot_x[i]= i//3
    plot_y[i]= i%3
    pose_puber[i]=rospy.Publisher(uav_type+'_'+str(i)+'/mavros/local_position/pose', PoseStamped, queue_size=2)
    vel_puber[i]=rospy.Publisher(uav_type+'_'+str(i)+'/mavros/local_position/velocity_local', TwistStamped, queue_size=2)


def cmd_vel_callback(msg,id):
    local_vel[id].twist=msg
    plot_x[id]+=step_time*local_vel[id].twist.linear.x
    plot_y[id]+=step_time*local_vel[id].twist.linear.y
    plot_z[id]+=step_time*local_vel[id].twist.linear.z


rospy.init_node('simple_3d_simulator')
rate = rospy.Rate(1/step_time)

for i in range(uav_num):
    rospy.Subscriber('/xtdrone/'+uav_type+'_'+str(i)+'/cmd_vel_flu', Twist, cmd_vel_callback,i) 
    rospy.Subscriber('/xtdrone/'+uav_type+'_'+str(i)+'/cmd_vel_enu', Twist, cmd_vel_callback,i)    


while not rospy.is_shutdown():
    for i in range(uav_num):
        local_pose=PoseStamped()
        local_pose.pose.position.x=plot_x[i]
        local_pose.pose.position.y=plot_y[i]
        local_pose.pose.position.z=plot_z[i]    
        pose_puber[i].publish(local_pose)
        vel_puber[i].publish(local_vel[i])
    rate.sleep()
