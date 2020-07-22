import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import sys 
sys.path.append('/home/robin/catkin_ws/devel/lib/python2.7/dist-packages')
from darknet_ros_msgs.msg import BoundingBoxes
import time
import math

def darknet_callback(data):
    global twist, cmd, target_height_mask, target_height
    find = False
    for target in data.bounding_boxes:
        if(target.id==0):
            print('find human')
            z = height / math.cos(math.radians(45))
            y_error=(x_center-(target.xmax+target.xmin)/2)*z/(fx+x_center)
            x_error=(y_center-(target.ymax+target.ymin)/2-y_center*y_error/z)*z/fy
            #print(x_error,y_error,height)
            twist.linear.x = Kp_xy*x_error
            twist.linear.y = Kp_xy*y_error
            twist.linear.z = Kp_z*(target_height-height)
            cmd = ''
            find = True
    if not find:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        cmd = 'HOVER'
        
def local_pose_callback(data):
    global height, target_height, target_set
    height = data.pose.position.z    
    if not target_set:
        target_height = height     
        target_set = True    
            
if __name__ == "__main__":
    height = 0  
    target_height = 0
    target_set = False
    twist = Twist()
    cmd = String()
    x_center=752/2
    y_center=480/2
    fx = 240
    fy = 240
    Kp_xy = 0.8
    Kp_z = 1
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('yolo_human_tracking')
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback)
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback)
    vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=2)
    cmd_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd', String, queue_size=2)
    rate = rospy.Rate(60) 
    while(True):
        rate.sleep()
        vel_pub.publish(twist)
        cmd_pub.publish(cmd)
        
