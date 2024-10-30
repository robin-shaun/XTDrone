import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import sys
from gazebo_msgs.msg import ModelStates

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
multi_pose_pub = [None]*vehicle_num
multi_odom_pub = [None]*vehicle_num
multi_speed_pub = [None]*vehicle_num
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_odom = [Odometry() for i in range(vehicle_num)]
multi_speed = [Vector3Stamped() for i in range(vehicle_num)]

def gazebo_model_state_callback(msg):
    for vehicle_id in range(vehicle_num):
        id = msg.name.index(vehicle_type+'_'+str(vehicle_id))
        multi_local_pose[vehicle_id].header.stamp = rospy.Time().now()
        multi_local_pose[vehicle_id].header.frame_id = 'map'
        multi_local_pose[vehicle_id].pose = msg.pose[id]
        multi_odom[vehicle_id].header.stamp = rospy.Time().now()
        multi_odom[vehicle_id].header.frame_id = 'map'
        multi_odom[vehicle_id].child_frame_id = "base_link"
        multi_odom[vehicle_id].pose.pose = msg.pose[id]
        multi_odom[vehicle_id].twist.twist = msg.twist[id]
        multi_speed[vehicle_id].header.stamp = rospy.Time().now()
        multi_speed[vehicle_id].header.frame_id = 'map'
        multi_speed[vehicle_id].vector = msg.twist[id]

if __name__ == '__main__':
    rospy.init_node(vehicle_type+'_get_pose_groundtruth')
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
    for i in range(vehicle_num):
        multi_pose_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        multi_odom_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_odom/odom', Odometry, queue_size=1)
        multi_speed_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_speed/speed', Vector3Stamped, queue_size=1)
        print("Get " + vehicle_type + "_" + str(i) + " groundtruth pose")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            multi_pose_pub[i].publish(multi_local_pose[i])
            multi_speed_pub[i].publish(multi_speed[i])
            multi_odom_pub[i].publish(multi_odom[i])
        try:
            rate.sleep()
        except:
            continue

