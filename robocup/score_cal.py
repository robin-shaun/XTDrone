import rospy
import sys
from std_msgs.msg import String,Int16
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from gazebo_msgs.srv import DeleteModel,GetModelState

err_threshold = 1
coordx_bias = 3
coordy_bias = 9

def actor_info_callback(msg):
    global target_finish, start_time, time_usage, score, count_flag
    actor_pos = get_model_state('actor_' + str(msg.id), 'ground_plane').pose.position
    if (msg.x-actor_pos.x+coordx_bias)**2+(msg.y-actor_pos.y+coordy_bias)**2<err_threshold**2:
        if not count_flag[msg.id]:
            count_flag[msg.id] = True
            find_time = rospy.Time.now().secs
        elif rospy.Time.now().secs - find_time >= 15:
            target_finish += 1
            del_model('actor_'+str(msg.id))
            time_usage = rospy.Time.now().secs - start_time
            print('actor_'+str(msg.id)+'is OK')
            print('Time usage:', time_usage)

            # calculate score
            if target_finish == 6:
                score = (800 - time_usage) - sensor_cost * 3e-3
                print("Mission finished")
                sys.exit()
            else:
                score = (1 + target_finish) * 2e2 - sensor_cost * 3e-3
            print('score:',score)
    else:
        count_flag[msg.id] = False

if __name__ == "__main__":
    actor_num = 6
    left_actors = range(actor_num)
    count_flag = [False] * actor_num
    rospy.init_node('score_cal')
    del_model = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState) 

    score_pub = rospy.Publisher("score",Int16,queue_size=2)  
    left_actors_pub = rospy.Publisher("left_actors",String,queue_size=2)
    actor_info_sub = [None] * actor_num
    for actor_id in range(actor_num):
        actor_info_sub = rospy.Subscriber("/actor_"+str(actor_id)+"_info",ActorInfo,actor_info_callback)

    # sensor cost
    mono_cam = 1
    stereo_cam =0
    laser1d = 0
    laser2d = 0
    laser3d = 0
    gimbal = 1
    sensor_cost = mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2+ laser2d * 5e3 + laser3d * 2e4 + gimbal * 2e2
    
    target_finish = 0
    time_usage = 10
    score = (1 + target_finish) * 2e2 - sensor_cost * 3e-3

    rate = rospy.Rate(10)
    start_time = rospy.Time.now().secs

    while not rospy.is_shutdown():
        score_pub.publish(score)
        rate.sleep()



