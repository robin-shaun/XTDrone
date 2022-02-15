import rospy
from geometry_msgs.msg import PoseStamped
import math
from pyquaternion import Quaternion
import sys
import tf

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
multi_pose_pub = [None]*vehicle_num
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
quaternion = tf.transformations.quaternion_from_euler(-math.pi/2, 0, -math.pi/2)
q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

def vision_callback(data, i):
    multi_local_pose[i].header.frame_id = 'world'
    multi_local_pose[i].pose.position.x = data.pose.position.x
    multi_local_pose[i].pose.position.y = data.pose.position.y
    multi_local_pose[i].pose.position.z = data.pose.position.z
    q_= Quaternion([data.pose.orientation.w,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z])
    q_ = q_*q
    multi_local_pose[i].pose.orientation.w = q_[0]
    multi_local_pose[i].pose.orientation.x = q_[1]
    multi_local_pose[i].pose.orientation.y = q_[2]
    multi_local_pose[i].pose.orientation.z = q_[3]
   
if __name__ == '__main__':
    rospy.init_node(vehicle_type+'_ego_transfer')
    for i in range(vehicle_num):
        rospy.Subscriber(vehicle_type+'_'+str(i)+'/mavros/vision_pose/pose', PoseStamped, vision_callback, i,queue_size=1)
        multi_pose_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/camera_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            multi_local_pose[i].header.stamp = rospy.Time.now()
            multi_pose_pub[i].publish(multi_local_pose[i])
        try:
            rate.sleep()
        except:
            continue


