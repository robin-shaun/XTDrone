import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
import sys
import tf
def get_odom():
    try:
        handle = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        response = handle('iris','ground_plane')
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    rospy.init_node('get_pose_groundtruth')
    if sys.argv[1] == 'groundtruth':
        pose_pub = rospy.Publisher("/pose_groundtruth", PoseStamped, queue_size=2)
    else:
        pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
    local_pose = PoseStamped()
    local_pose.header.frame_id = 'map'
    '''
    tf transfer should be improved
    quaternion = tf.transformations.quaternion_from_euler(0, 0, -float(sys.argv[1]))
    q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
    '''
    rate = rospy.Rate(100)
    while True:
        odom= get_odom()
        local_pose.header.stamp = rospy.Time.now()
        local_pose.pose = odom.pose
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

