import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import sys
from gazebo_msgs.msg import ModelStates

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
laser_slam_type = sys.argv[3]
rospy.init_node(vehicle_type+vehicle_id+'_'+laser_slam_type+'_laser_transfer')
pose_pub = rospy.Publisher(vehicle_type+'_'+ vehicle_id+"/mavros/vision_pose/pose", PoseStamped, queue_size=1)
tfBuffer = Buffer()
tflistener = TransformListener(tfBuffer)
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
hector = PoseStamped()
height = 0


def hector_callback(data):
    global hector
    hector = data

def gazebo_model_state_callback(msg):
    global height
    id = msg.name.index(vehicle_type+'_'+str(vehicle_id))
    height = msg.pose[id].position.z
    
def hector_slam():
    global local_pose, height
    pose2d_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/pose", PoseStamped, hector_callback,queue_size=1)
    rate = rospy.Rate(100)
    while True:
        local_pose = hector
        local_pose.pose.position.z = height
        pose_pub.publish(local_pose)
        rate.sleep()
        
def cartographer():
    global local_pose, height
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            tfstamped = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        except:
            continue
        local_pose.header.stamp = rospy.Time().now()
        local_pose.pose.position = tfstamped.transform.translation
        local_pose.pose.position.z = height
        local_pose.pose.orientation = tfstamped.transform.rotation
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
        local_pose.pose.position = tfstamped.transform.translation
        local_pose.pose.orientation = tfstamped.transform.rotation
        pose_pub.publish(local_pose)
        rate.sleep()
    
if __name__ == '__main__':
    if laser_slam_type == 'hector':
        gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
        hector_slam()
    elif laser_slam_type == 'cartographer':
        gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
        cartographer()
    elif laser_slam_type == 'aloam':
        aloam()
    else:
        print('input error')
        

    
