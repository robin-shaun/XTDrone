import rospy
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from gazebo_msgs.srv import GetModelState
import time

if __name__ == "__main__":
    rospy.init_node("test")
    actors_pos = [None] * 6
    red1_actorinfo = ActorInfo(cls='red',x=10.0,y=10.0)
    green_actorinfo = ActorInfo(cls='green',x=10.0,y=10.0)
    blue_actorinfo = ActorInfo(cls='blue',x=10.0,y=10.0)
    white_actorinfo = ActorInfo(cls='white',x=10.0,y=10.0)
    red2_actorinfo = ActorInfo(cls='red',x=10.0,y=10.0)
    brown_actorinfo = ActorInfo(cls='brown',x=10.0,y=10.0)
    test_pub_red2 = rospy.Publisher("/actor_red2_info",ActorInfo,queue_size=2)
    test_pub_green = rospy.Publisher("/actor_green_info",ActorInfo,queue_size=2)
    test_pub_red1 = rospy.Publisher("/actor_red1_info",ActorInfo,queue_size=2)
    test_pub_brown = rospy.Publisher("/actor_brown_info",ActorInfo,queue_size=2)
    test_pub_blue = rospy.Publisher("/actor_blue_info",ActorInfo,queue_size=2)
    test_pub_white = rospy.Publisher("/actor_white_info",ActorInfo,queue_size=2)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
    rate = rospy.Rate(2)
    time.sleep(1)
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        for i in range(6):
            actors_pos[i] = get_model_state('actor_' + str(i), 'ground_plane').pose.position
        red1_actorinfo.x = actors_pos[5].x
        red1_actorinfo.y = actors_pos[5].y
        green_actorinfo.x = actors_pos[0].x
        green_actorinfo.y = actors_pos[0].y
        red2_actorinfo.x = actors_pos[4].x
        red2_actorinfo.y = actors_pos[4].y
        white_actorinfo.x = actors_pos[3].x
        white_actorinfo.y = actors_pos[3].y
        blue_actorinfo.x = actors_pos[1].x
        blue_actorinfo.y = actors_pos[1].y
        brown_actorinfo.x = actors_pos[2].x
        brown_actorinfo.y = actors_pos[2].y
        if rospy.get_time() - start_time < 20:
            test_pub_red2.publish(red2_actorinfo)
        if rospy.get_time() - start_time > 5:
            test_pub_green.publish(green_actorinfo)
        if rospy.get_time() - start_time > 6:
            test_pub_blue.publish(blue_actorinfo)
        if rospy.get_time() - start_time > 10:
            test_pub_brown.publish(brown_actorinfo)
        if rospy.get_time() - start_time > 15:
            test_pub_red1.publish(red1_actorinfo)
        if rospy.get_time() - start_time > 20:
            test_pub_white.publish(white_actorinfo)
        rate.sleep()