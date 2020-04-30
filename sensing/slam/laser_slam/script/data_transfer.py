import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import tf
import sys

rospy.init_node('laser_slam_data_transfer')
pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
rate = rospy.Rate(100)
local_pose = PoseStamped()
laser_scan = Pose2D()

def odm_groundtruth_callback(msg):
    local_pose.header = msg.header
    local_pose.pose.position.z  = msg.pose.pose.position.z

def callback(data):
    global laser_scan
    laser_scan = data
    
def laser_scan_matcher():
    pose2d_sub = rospy.Subscriber("/pose2D", Pose2D, callback)
    while True:
        local_pose.header.stamp = rospy.Time.now()
        local_pose.pose.position.x = laser_scan.x
        local_pose.pose.position.y = laser_scan.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, laser_scan.theta)
        local_pose.pose.orientation.x = quaternion[0]
        local_pose.pose.orientation.y = quaternion[1]
        local_pose.pose.orientation.z = quaternion[2]
        local_pose.pose.orientation.w = quaternion[3]
        pose_pub.publish(local_pose)
        rate.sleep()

def cartographer2D():
    listener = tf.TransformListener()
    while True:
        try:
            translation,rotation = listener.lookupTransform("base_link","odom",rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        local_pose.pose.position.x = translation[0]
        local_pose.pose.position.y = translation[1]
        local_pose.pose.orientation.y = rotation[2]
        local_pose.pose.orientation.z = rotation[3]
        pose_pub.publish(local_pose)
        rate.sleep()
    
if __name__ == '__main__':
    odom_groundtruth_sub = rospy.Subscriber('/xtdrone/ground_truth/odom', Odometry, odm_groundtruth_callback)
    try:
        laser_slam_type = sys.argv[1]
    except:
        print('You should choose from laser_scan_matcher, cartographer2D and cartographer3D.')
    if laser_slam_type == 'laser_scan_matcher':
        laser_scan_matcher()
    elif laser_slam_type == 'cartographer2D':
        cartographer2D()
    elif laser_slam_type == 'cartographer3D':
        print('You should choose from laser_scan_matcher, cartographer2D and cartographer3D.')
    else:
        print('You should choose from laser_scan_matcher, cartographer2D and cartographer3D.')
        

    
