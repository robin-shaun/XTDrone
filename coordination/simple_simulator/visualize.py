import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   
import matplotlib.animation as animation              
import numpy as np 
import rospy
from geometry_msgs.msg import Twist,Pose,PoseStamped,TwistStamped
from gazebo_msgs.srv import GetModelState
import sys

uav_num = int(sys.argv[1])

step_time=0.005

pose_puber=[None]*uav_num
vel_puber=[None]*uav_num

plot_x=[0]*(uav_num)
plot_y=[0]*(uav_num)
plot_z=[0]*(uav_num)


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

def pose_sub_callback(msg,id):

    plot_x[id]=msg.pose.position.x
    plot_y[id]=msg.pose.position.y
    plot_z[id]=msg.pose.position.z

rospy.init_node('visualize')
rate = rospy.Rate(1/step_time)

for i in range(uav_num):
    rospy.Subscriber('/uav'+str(i+1)+'/mavros/local_position/pose', PoseStamped, pose_sub_callback,i)  

try:
    while not rospy.is_shutdown():
        init()
        ax.scatter(plot_x, plot_y, plot_z, s=50, marker="o")
        ax.tick_params(axis="x", labelsize=20)
        ax.tick_params(axis="y", labelsize=20)
        ax.tick_params(axis="z", labelsize=20)
        plt.pause(step_time)
        ax.cla()
        rate.sleep()
except KeyboardInterrupt:
        plt.ioff()