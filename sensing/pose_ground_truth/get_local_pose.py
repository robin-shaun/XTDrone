import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
import sys
import tf

local_pose = PoseStamped()

def odm_groundtruth_callback(msg):
    local_pose.header = msg.header
    local_pose.pose = msg.pose.pose

if __name__ == '__main__':
    rospy.init_node('get_pose_groundtruth')
    odom_groundtruth_sub = rospy.Subscriber('/xtdrone/ground_truth/odom', Odometry, odm_groundtruth_callback)
    pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
    '''
    tf transfer should be improved
    quaternion = tf.transformations.quaternion_from_euler(0, 0, -float(sys.argv[1]))
    q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
    '''
    rate = rospy.Rate(100)
    while True:
        '''
        q_= Quaternion([local_pose.pose.orientation.w,local_pose.pose.orientation.x,local_pose.pose.orientation.y,local_pose.pose.orientation.z])
        q_ = q_*q
        local_pose.pose.orientation.w = q_[0]
        local_pose.pose.orientation.x = q_[1]
        local_pose.pose.orientation.y = q_[2]
        local_pose.pose.orientation.z = q_[3]
        '''
        pose_pub.publish(local_pose)
        rate.sleep()

