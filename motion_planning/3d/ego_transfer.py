import rospy
from geometry_msgs.msg import PoseStamped
import math
from pyquaternion import Quaternion
import tf
import sys

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
local_pose = PoseStamped()
local_pose.header.frame_id = 'world'
quaternion = tf.transformations.quaternion_from_euler(-math.pi/2, 0, -math.pi/2)
q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

def vision_callback(data):    
    local_pose.pose.position.x = data.pose.position.x
    local_pose.pose.position.y = data.pose.position.y
    local_pose.pose.position.z = data.pose.position.z
    q_= Quaternion([data.pose.orientation.w,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z])
    q_ = q_*q
    local_pose.pose.orientation.w = q_[0]
    local_pose.pose.orientation.x = q_[1]
    local_pose.pose.orientation.y = q_[2]
    local_pose.pose.orientation.z = q_[3]
    
rospy.init_node(vehicle_type+"_"+vehicle_id+'_ego_transfer')
rospy.Subscriber(vehicle_type+"_"+vehicle_id+"/mavros/vision_pose/pose", PoseStamped, vision_callback,queue_size=1)
position_pub = rospy.Publisher(vehicle_type+"_"+vehicle_id+"/camera_pose", PoseStamped, queue_size=1)
rate = rospy.Rate(60) 

while not rospy.is_shutdown():
    local_pose.header.stamp = rospy.Time.now()
    position_pub.publish(local_pose) 
    rate.sleep()
  
