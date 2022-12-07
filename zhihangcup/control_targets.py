import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetLinkState

def pose_publisher():
    model_state_pub = rospy.Publisher('/gazebo/set_model_states', ModelStates, queue_size=1)
    relative_pose_pub = rospy.Publisher("/gazebo/relative_pose", Pose, queue_size=1)
    poses_msg = ModelStates()
    poses_msg.name = [None] * 3
    poses_msg.pose = [Pose()for i in range(3)]
    poses_msg.twist = [Twist()for i in range(3)]
    poses_msg.name[0] = 'target_red'
    poses_msg.name[1] = 'target_blue'
    poses_msg.name[2] = 'target_green'

    poses_msg.pose[0].position.x  = -8
    poses_msg.pose[0].position.y = -8.5
    poses_msg.pose[0].position.z = 1.5
    
    poses_msg.pose[1].position.x  = 11
    poses_msg.pose[1].position.y = 2.5
    poses_msg.pose[1].position.z = 1.5

    poses_msg.pose[2].position.x  = 13.5
    poses_msg.pose[2].position.y = -8
    poses_msg.pose[2].position.z = 1.5
    poses_msg.pose[2].orientation.z = 0.707106
    poses_msg.pose[2].orientation.w = 0.707106

    f = 10.0
    v = 0.0
    a = 0.5
    i = 0
    period_1 = 2
    period_2 = period_1 + 4
    period_3 = period_2 + 2 + 2
    period_4 = period_3 + 4
    period_total = period_4 + 2
    rate = rospy.Rate(f)
    while not rospy.is_shutdown():
        if (i%(period_total*f)<period_1*f):
            v = v + a/f
        elif (i%(period_total*f)<period_2*f):
            v = 1.0
        elif (i%(period_total*f)<period_3*f):
            v = v - a/f
        elif (i%(period_total*f)<period_4*f):
            v = -1.0
        else:
            v = v + a/f
        poses_msg.pose[0].position.y = poses_msg.pose[0].position.y + v / f
        poses_msg.pose[1].position.y = poses_msg.pose[1].position.y + v / f
        
        if(id==3):
            poses_msg.pose[2].position.y = poses_msg.pose[2].position.y + v / f
        else:
            poses_msg.pose[2].position.x = poses_msg.pose[2].position.x + v / f
        

        model_state_pub.publish(poses_msg)
        i = i + 1
        try:
            response = get_link_state('iris_0::realsense_camera::link', 'target_green::link')
            relative_pose = response.link_state.pose
            relative_pose_pub.publish(relative_pose)
        except:
            continue
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('control_targets')
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    pose_publisher()