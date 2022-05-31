import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
import sys

def pose_publisher():
    vehicle_type = sys.argv[1]
    row_num = int(sys.argv[2])
    column_num = int(sys.argv[3])
    vehicle_num = row_num * column_num
    pub = rospy.Publisher('gazebo/set_model_states', ModelStates, queue_size=1)
    poses_msg = ModelStates()
    poses_msg.name = [None] * vehicle_num
    poses_msg.pose = [Pose()for i in range(vehicle_num)]
    poses_msg.twist = [Twist()for i in range(vehicle_num)]
    for i in range(vehicle_num):
        poses_msg.name[i] = vehicle_type + '_' + str(i)
        poses_msg.pose[i].position.x  = i / row_num
        poses_msg.pose[i].position.y = i % row_num
        poses_msg.pose[i].position.z = 3
    f = 10
    v = 0.5
    rate = rospy.Rate(f)
    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            poses_msg.pose[i].position.x = poses_msg.pose[i].position.x + 0.5 / f
        pub.publish(poses_msg)
        rate.sleep()
if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
          pose_publisher()
      except rospy.ROSInterruptException:
          pass