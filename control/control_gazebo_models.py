import rospy
from gazebo_msgs.msg import ModelState

def pose_publisher():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
    pose_msg = ModelState()
    pose_msg.model_name = 'iris_0'
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
           pose_msg.pose.position.x = -2
           pose_msg.pose.position.y = 7.5
           pose_msg.pose.position.z = 1
           #same for orientation
           pub.publish(pose_msg)
           rate.sleep()

if __name__ == '__main__':
      rospy.init_node('pose_publisher')
      try:
          pose_publisher()
      except rospy.ROSInterruptException:
          pass