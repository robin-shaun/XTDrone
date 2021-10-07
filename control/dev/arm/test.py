import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist, Pose, TransformStamped

def link_states_callback(msg):
    try:
        box_id = msg.name.index("box::base_link")
        camera_id = msg.name.index("iris_0::gripper_base_link")
        box_pose = msg.pose[box_id]
        camera_pose = msg.pose[camera_id]
        print('box: ', box_pose)
        print('camera: ', camera_pose)
    except ValueError:
        pass 

rospy.init_node('test')
rospy.Subscriber("/gazebo/link_states", LinkStates, link_states_callback)

rate = rospy.Rate(20) 
        
while not rospy.is_shutdown():
            rate.sleep()