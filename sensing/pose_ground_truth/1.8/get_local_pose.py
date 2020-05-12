import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import sys
import tf

local_pose = PoseStamped()

def odm_groundtruth_callback(msg):
    local_pose.header = msg.header
    local_pose.pose = msg.pose.pose

if __name__ == '__main__':
    rospy.init_node('get_pose_groundtruth')
    odom_groundtruth_sub = rospy.Subscriber('/xtdrone/ground_truth/odom', Odometry, odm_groundtruth_callback)
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=2)
    rate = rospy.Rate(100)
    while True:
        pose_pub.publish(local_pose)
        rate.sleep()

