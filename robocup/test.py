import rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

if __name__ == "__main__":
    rospy.init_node("test")
    red_actorinfo = ActorInfo(cls='red',x=10.0,y=10.0)
    green_actorinfo = ActorInfo(cls='green',x=10.0,y=10.0)
    test_pub_red2 = rospy.Publisher("/actor_red2_info",ActorInfo,queue_size=2)
    test_pub_green = rospy.Publisher("/actor_green_info",ActorInfo,queue_size=2)
    rospy.Rate(1)
    while not rospy.is_shutdown():
        test_pub_red2.publish(red_actorinfo)
        test_pub_green.publish(green_actorinfo)