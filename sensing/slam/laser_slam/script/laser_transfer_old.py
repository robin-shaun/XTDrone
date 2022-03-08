import rospy
from geometry_msgs.msg import PoseStamped, Pose2D
import tf
import sys
from gazebo_msgs.msg import ModelStates

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
laser_slam_type = sys.argv[3]
rospy.init_node(vehicle_type+vehicle_id+'_'+laser_slam_type+'_laser_transfer')
pose_pub = rospy.Publisher(vehicle_type+'_'+ vehicle_id+"/mavros/vision_pose/pose", PoseStamped, queue_size=2)
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
laser_scan = Pose2D()
    
def gazebo_model_state_callback(msg):
    global local_pose
    id = msg.name.index(vehicle_type+'_'+str(vehicle_id))
    local_pose.pose.position.z = msg.pose[id].position.z

def laser_scan_matcher_callback(data):
    global laser_scan
    laser_scan = data
    
def laser_scan_matcher():
    global local_pose
    pose2d_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/pose2D", Pose2D, laser_scan_matcher_callback)
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
    
if __name__ == '__main__':
    
    if laser_slam_type == '2d':
        gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
        laser_scan_matcher()
    else:
        print('input error')