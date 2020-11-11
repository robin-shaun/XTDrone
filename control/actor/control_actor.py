import rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
import sys

# These areas are houses
# forbidden_area =[[[-33, -21], [18, 33]], [[5, 20], [11, 25]], [[54, 68], [15, 31]], [[71, 83], [9, 17]],
#             [[88, 100], [11, 16]], [[78, 95], [22, 35]], [[53, 70], [-35, -27]], [[-6, 7], [-18, -10]],
#             [[20, 38], [-18, -10]], [[-5, 5], [-34, -21]], [[-28, -23], [-28, -15]], [[-36, -23], [-35, -30]],
#             [[-36, -31], [-26, -13]], [[15, 18], [-16, -12]]]

# The size of the scene is [[-50,150],[-50,50]]

if __name__ == "__main__":
    actor_id = sys.argv[1]
    rospy.init_node('control_actor_'+actor_id)
    cmd_motion = ActorMotion(x=float(sys.argv[2]),y=int(sys.argv[3]),v=int(sys.argv[4]))
    cmd_motion_pub = rospy.Publisher('/actor_' + actor_id + '/cmd_motion', ActorMotion, queue_size=2)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        cmd_motion_pub.publish(cmd_motion)
        rate.sleep()
