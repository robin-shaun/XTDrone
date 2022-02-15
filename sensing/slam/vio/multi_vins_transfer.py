import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from pyquaternion import Quaternion
import sys
import tf
import numpy as np

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
multi_pose_pub = [None]*vehicle_num
multi_odom_pub = [None]*vehicle_num
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_odom = [Odometry() for i in range(vehicle_num)]
quaternion = tf.transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

bias = np.array([[-5,-2,0],[5,4,0],[-7,-8,0],[-9,8,0]])

def multi_vins_callback(data, i):
    multi_local_pose[i].header.frame_id = 'world'
    multi_local_pose[i].pose.position.x = data.pose.pose.position.x + bias[i,0]
    multi_local_pose[i].pose.position.y = data.pose.pose.position.y + bias[i,1]
    multi_local_pose[i].pose.position.z = data.pose.pose.position.z + bias[i,2]
    q_= Quaternion([data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])
    q_ = q_*q
    multi_local_pose[i].pose.orientation.w = q_[0]
    multi_local_pose[i].pose.orientation.x = q_[1]
    multi_local_pose[i].pose.orientation.y = q_[2]
    multi_local_pose[i].pose.orientation.z = q_[3]

def odom_callback(data,i):
    multi_odom[i].header.frame_id = 'world'
    multi_odom[i].child_frame_id = 'world'
    multi_odom[i].pose.pose.position.x = data.pose.pose.position.x + bias[i,0]
    multi_odom[i].pose.pose.position.y = data.pose.pose.position.y + bias[i,1]
    multi_odom[i].pose.pose.position.z = data.pose.pose.position.z + bias[i,2]
    multi_odom[i].pose.pose.orientation.x = data.pose.pose.orientation.x
    multi_odom[i].pose.pose.orientation.y = data.pose.pose.orientation.y
    multi_odom[i].pose.pose.orientation.z = data.pose.pose.orientation.z
    multi_odom[i].pose.pose.orientation.w = data.pose.pose.orientation.w
    multi_odom[i].twist.twist.linear.x = data.twist.twist.linear.x
    multi_odom[i].twist.twist.linear.y = data.twist.twist.linear.y
    multi_odom[i].twist.twist.linear.z = data.twist.twist.linear.z
            
if __name__ == '__main__':
    rospy.init_node(vehicle_type+'_vins_transfer')
    for i in range(vehicle_num):
        rospy.Subscriber(vehicle_type+'_'+str(i)+'/vins_estimator/camera_pose', Odometry, multi_vins_callback, i,queue_size=1)
        rospy.Subscriber(vehicle_type+'_'+str(i)+'/vins_estimator/odometry', Odometry, odom_callback, i,queue_size=1)
        multi_pose_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        multi_odom_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/odometry', Odometry, queue_size=1)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            multi_local_pose[i].header.stamp = rospy.Time.now()
            multi_pose_pub[i].publish(multi_local_pose[i])
            multi_odom_pub[i].publish(multi_odom[i])
        try:
            rate.sleep()
        except:
            continue


