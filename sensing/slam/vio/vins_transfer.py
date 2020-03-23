import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from pyquaternion import Quaternion

def vins_callback(data):
    #rospy.loginfo(str(data.pose.position.x)+','+str(data.pose.position.y)+','+str(data.pose.position.z))
    global posx, posy, posz, orientation, qx, qy, qz, qw
    posx = data.pose.pose.position.x
    posy = data.pose.pose.position.y
    posz = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w

posx, posy, posz = 0, 0, 0
orientation = Quaternion(1,0,0,0)
count = 0
posx, posy, posz, qx, qy ,qz ,qw = 0, 0, 0, 0, 0, 0, 1
rospy.init_node('transfer')
rospy.Subscriber("/vins_estimator/camera_pose", Odometry, vins_callback)
 # spin() simply keeps python from exiting until this node is stopped
position_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2) #creates a publisher object
setpoint_msg = PoseStamped()
rate = rospy.Rate(60) #the rate will take care of updating the publisher at a specified speed - 50hz in this case


while True:
    setpoint_msg.header.stamp = rospy.Time.now()
    #rospy.loginfo(str(posx)+','+str(posy)+','+str(posz))
    setpoint_msg.header.frame_id = 'map'
    setpoint_msg.pose.position.x = posx
    setpoint_msg.pose.position.y = posy
    setpoint_msg.pose.position.z = posz
    setpoint_msg.pose.orientation.x = qx
    setpoint_msg.pose.orientation.y = qy
    setpoint_msg.pose.orientation.z = qz
    setpoint_msg.pose.orientation.w = qw
    position_pub.publish(setpoint_msg) 
    rate.sleep()
#creates a message object of type pose stamped #http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
  
