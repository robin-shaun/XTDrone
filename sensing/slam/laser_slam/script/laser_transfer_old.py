import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import sys

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
laser_slam_type = sys.argv[3]
rospy.init_node(vehicle_type+vehicle_id+'_'+laser_slam_type+'_laser_transfer')
pose_pub = rospy.Publisher(vehicle_type+'_'+ vehicle_id+"/mavros/vision_pose/pose", PoseStamped, queue_size=1)
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
laser_scan = Pose2D()

def odm_groundtruth_callback(msg):
    global local_pose
    local_pose.header.stamp = msg.header.stamp
    local_pose.pose.position.z  = msg.pose.pose.position.z

def laser_scan_matcher_callback(data):
    global laser_scan
    laser_scan = data
    
def laser_scan_matcher():
    global local_pose
    pose2d_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/pose2D", Pose2D, laser_scan_matcher_callback,queue_size=1)
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
        pose_pub.publish(local_pose)
        rate.sleep()
        
def aloam():
    global local_pose
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            tfstamped = tfBuffer.lookup_transform('camera_init', 'aft_mapped', rospy.Time(0))
        except:
            continue
        local_pose.header.stamp = rospy.Time().now()
        local_pose.header.frame_id = 'map'
        local_pose.pose.position = tfstamped.transform.translation
        local_pose.pose.orientation = tfstamped.transform.rotation
        pose_pub.publish(local_pose)
        rate.sleep()
    
if __name__ == '__main__':
    
    if laser_slam_type == '2d':
        odom_groundtruth_sub = rospy.Subscriber('/xtdrone/'+vehicle_type+'_'+ vehicle_id+'/ground_truth/odom', Odometry, odm_groundtruth_callback)
        laser_scan_matcher()
    elif laser_slam_type == '3d':
        aloam()
    else:
        print('input error')
        

    
