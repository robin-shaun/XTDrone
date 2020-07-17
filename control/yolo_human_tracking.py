import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys 
sys.path.append('/home/robin/catkin_ws/devel/lib/python2.7/dist-packages')
from darknet_ros_msgs.msg import BoundingBoxes
import time
import math

def darknet_callback(data):
    global twist, cmd
    find = False
    for target in data.bounding_boxes:
        if(target.id==0):
            print('find human')
            x_error=y_center-(target.ymax+target.ymin)/2
            y_error=x_center-(target.xmax+target.xmin)/2
            twist.linear.x = Kp_linear*x_error
            twist.angular.z = Kp_angular*math.atan(y_error/x_error)
            cmd = ''
            find = True
    if not find:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd = 'HOVER'
                
            
if __name__ == "__main__":  
    twist = Twist()
    cmd = String()
    Kp_linear=0.006
    Kp_angular=0.2/math.pi
    x_center=752/2
    y_center=480/2
    vehicle_type = sys.argv[1]
    rospy.init_node('yolo_human_tracking')
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback)
    vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_0/cmd_vel_flu', Twist, queue_size=2)
    cmd_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_0/cmd', String, queue_size=2)
    rate = rospy.Rate(60) 
    while(True):
        rate.sleep()
        vel_pub.publish(twist)
        cmd_pub.publish(cmd)
        
