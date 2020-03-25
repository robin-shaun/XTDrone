import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   
import matplotlib.animation as animation              
import numpy as np 
import rospy
from geometry_msgs.msg import Twist,Pose,PoseStamped,TwistStamped
from gazebo_msgs.srv import GetModelState
import sys

use_1_8 = 1

#uav_num=9
uav_num = int(sys.argv[1])

step_time=0.01

pose_puber=[None]*uav_num
vel_puber=[None]*uav_num

plot_x=[0]*(uav_num)
plot_y=[0]*(uav_num)
plot_z=[0]*(uav_num)
local_vel = [TwistStamped()]*(uav_num)

for i in range(uav_num):
    uav_id=i+use_1_8
    plot_x[i]= i//3
    plot_y[i]= i%3
    pose_puber[i]=rospy.Publisher('/uav'+str(uav_id)+'/mavros/local_position/pose', PoseStamped, queue_size=10)
    vel_puber[i]=rospy.Publisher('/uav'+str(uav_id)+'/mavros/local_position/velocity_local', TwistStamped, queue_size=10)

fig = plt.figure()
plt.ion()
ax = Axes3D(fig)
label_lim = 20

def scroll_call_back(event):
    global label_lim
    if event.button == 'up':
        label_lim+=2
        #print('up')
    elif event.button == 'down':
        label_lim=label_lim-2 if label_lim>1 else 1
        #print('down')


fig.canvas.mpl_connect('scroll_event', scroll_call_back)

def init():
    ax.set_xlim3d(-label_lim, label_lim)
    ax.set_ylim3d(-label_lim, label_lim)
    ax.set_zlim3d(-label_lim, label_lim)


def cmd_vel_callback(msg,id):
 #   accel=msg
 #   plot_x[id]+=step_time*accel.linear.x
  #  plot_y[id]+=step_time*accel.linear.y
  #  plot_z[id]+=step_time*accel.linear.z
    local_vel[id].twist=msg
    plot_x[id]+=step_time*local_vel[id].twist.linear.x
    plot_y[id]+=step_time*local_vel[id].twist.linear.y
    plot_z[id]+=step_time*local_vel[id].twist.linear.z


rospy.init_node('simple_3d_simulator')
rate = rospy.Rate(1/step_time)

for i in range(uav_num):
    uav_id=i+use_1_8
    rospy.Subscriber('/xtdrone/uav'+str(uav_id)+'/cmd_vel_flu', Twist, cmd_vel_callback,i) 
    rospy.Subscriber('/xtdrone/uav'+str(uav_id)+'/cmd_vel_enu', Twist, cmd_vel_callback,i)    

try:
    while not rospy.is_shutdown():
        for i in range(uav_num):
            local_pose=PoseStamped()
            local_pose.pose.position.x=plot_x[i]
            local_pose.pose.position.y=plot_y[i]
            local_pose.pose.position.z=plot_z[i]    
            pose_puber[i].publish(local_pose)
            vel_puber[i].publish(local_vel[i])
        init()
        ax.scatter(plot_x,plot_y,plot_z,marker="x")
        plt.pause(step_time)
        ax.cla()
        rate.sleep()
except KeyboardInterrupt:
        plt.ioff()