import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import sys

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
laser_slam_type = sys.argv[3]
rospy.init_node(vehicle_type+vehicle_id+'_'+laser_slam_type+'_laser_transfer')
pose_pub = rospy.Publisher(vehicle_type+'_'+ vehicle_id+"/mavros/vision_pose/pose", PoseStamped, queue_size=2)
tfBuffer = Buffer()
tflistener = TransformListener(tfBuffer)
local_pose = PoseStamped()
hector = PoseStamped()
height = 0

def odm_groundtruth_callback(msg):
    global height
    height = msg.pose.pose.position.z

def hector_callback(data):
    global hector
    hector = data
    
def hector_slam():
    global local_pose, height
    pose2d_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/pose", PoseStamped, hector_callback)
    rate = rospy.Rate(100)
    while True:
        local_pose = hector
        local_pose.pose.position.z = height
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
        hector_slam()
    elif laser_slam_type == '3d':
        aloam()
    else:
        print('input error')
        

    
