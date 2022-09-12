import rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
import sys

if __name__ == "__main__":
    actor_id = sys.argv[1]
    rospy.init_node('control_actor_'+actor_id)
    cmd_motion = ActorMotion(x=float(sys.argv[2]),y=float(sys.argv[3]),v=float(sys.argv[4]))
    cmd_motion_pub = rospy.Publisher('/running_' + actor_id + '/cmd_motion', ActorMotion, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        cmd_motion_pub.publish(cmd_motion)
        rate.sleep()
