import rospy
from geometry_msgs.msg import Twist
import sys 
sys.path.append('/home/robin/catkin_ws/devel/lib/python2.7/dist-packages')
from darknet_ros_msgs.msg import BoundingBoxes
import time
import math
twist = Twist()
def darknet_callback(data):
    for target in data.bounding_boxes:
        print(target.id)
        if(target.id==0):
            x_error=y_center-(target.ymax+target.ymin)/2
            y_error=x_center-(target.xmax+target.xmin)/2
            twist.linear.x = Kp_linear*x_error
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = Kp_angular*math.atan(y_error/x_error)

        else:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0         

Kp_linear=0.05
Kp_angular=0.2/math.pi
x_center=752/2
y_center=480/2
rospy.init_node('yolo_human_tracking')
rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback)
pub = rospy.Publisher('/xtdrone/cmd_vel_flu', Twist, queue_size=10)
rate = rospy.Rate(60) 
while(True):
    rate.sleep()
    pub.publish(twist)
