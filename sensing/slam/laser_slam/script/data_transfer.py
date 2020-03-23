import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
import tf

def get_odom():
    try:
        handle = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        response = handle('iris','ground_plane')
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def callback(data):
    global laser_scan
    laser_scan = data

if __name__ == '__main__':
    laser_scan = Pose2D()
    rospy.init_node('get_pose_groundtruth')
    pose2d_sub = rospy.Subscriber("/pose2D", Pose2D, callback)
    pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
    local_pose = PoseStamped()
    local_pose.header.frame_id = 'map'
    rate = rospy.Rate(100)
    while True:
        odom= get_odom()
        local_pose.header.stamp = rospy.Time.now()
        local_pose.pose.position.x = laser_scan.x
        local_pose.pose.position.y = laser_scan.y
        local_pose.pose.position.z = odom.pose.position.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, laser_scan.theta)
        local_pose.pose.orientation.x = quaternion[0]
        local_pose.pose.orientation.y = quaternion[1]
        local_pose.pose.orientation.z = quaternion[2]
        local_pose.pose.orientation.w = quaternion[3]
        pose_pub.publish(local_pose)
        rate.sleep()