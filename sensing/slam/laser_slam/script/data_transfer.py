import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry
import tf
import sys

rospy.init_node('laser_slam_data_transfer')
pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
laser_scan = Pose2D()

def odm_groundtruth_callback(msg):
    global local_pose
    local_pose.header.stamp = msg.header.stamp
    local_pose.pose.position.z  = msg.pose.pose.position.z
    
def odm_aloam_callback(msg):
    global local_pose
    local_pose.header.stamp = msg.header.stamp
    local_pose.pose = msg.pose.pose

def callback(data):
    global laser_scan
    laser_scan = data
    
def laser_scan_matcher():
    global local_pose
    pose2d_sub = rospy.Subscriber("/pose2D", Pose2D, callback)
    rate = rospy.Rate(100)
    while True:
        local_pose.header.stamp = rospy.Time.now()
        local_pose.pose.position.x = laser_scan.x
        local_pose.pose.position.y = laser_scan.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, laser_scan.theta)
        local_pose.pose.orientation.x = quaternion[0]
        local_pose.pose.orientation.y = quaternion[1]
        local_pose.pose.orientation.z = quaternion[2]
        local_pose.pose.orientation.w = quaternion[3]
        rate.sleep()

def cartographer2D():
    listener = tf.TransformListener()
    global local_pose
    rate = rospy.Rate(100)
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
        
def cartographer3D():
    listener = tf.TransformListener()
    global local_pose
    rate = rospy.Rate(100)
    while True:
        try:
            translation,rotation = listener.lookupTransform("base_link","odom",rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        local_pose.pose.position.x = translation[0]
        local_pose.pose.position.y = translation[1]
        local_pose.pose.position.z = translation[2]
        local_pose.pose.orientation.w = rotation[0]
        local_pose.pose.orientation.x = rotation[1]
        local_pose.pose.orientation.y = rotation[2]
        local_pose.pose.orientation.z = rotation[3]
        pose_pub.publish(local_pose)
        rate.sleep()
        
def aloam():
    global local_pose
    rate = rospy.Rate(100)
    while True:
        pose_pub.publish(local_pose)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        laser_slam_type = sys.argv[1]
    except:
        print('You should choose from laser_scan_matcher, cartographer2D and cartographer3D.')
    if laser_slam_type == 'laser_scan_matcher':
        odom_groundtruth_sub = rospy.Subscriber('/xtdrone/ground_truth/odom', Odometry, odm_groundtruth_callback)
        laser_scan_matcher()
    elif laser_slam_type == 'cartographer2D':
        odom_groundtruth_sub = rospy.Subscriber('/xtdrone/ground_truth/odom', Odometry, odm_groundtruth_callback)
        cartographer2D()
    elif laser_slam_type == 'cartographer3D':
        cartographer3D()
    elif laser_slam_type == 'aloam':
        odom_aloam_sub = rospy.Subscriber('/laser_odom_to_init', Odometry, odm_aloam_callback)
        aloam()
    else:
        print('You should choose from laser_scan_matcher, cartographer2D and cartographer3D.')
        

    
