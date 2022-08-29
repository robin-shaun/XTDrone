#coding: utf-8
import rospy
import sys
from std_msgs.msg import Int16,String
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from gazebo_msgs.srv import DeleteModel,GetModelState
import time
import cv2

uav_type = sys.argv[1]
actor_num = 6
uav_num = 6
err_threshold = 1
coordx_bias = 3
coordy_bias = 9
actor_id_dict = {'green':[0], 'blue':[1], 'brown':[2], 'white':[3], 'red':[4,5]}

# def state_callback(msg, vehicle_id):
#     global fall_detect, start_time
#     if rospy.get_time() - start_time > 60 and not msg.armed and fall_detect[vehicle_id] == 0:
#         fall_detect[vehicle_id] = 1
#         print('UAV_' + str(vehicle_id) + ': falled')

def actor_info_callback(msg):
    global target_finish, start_time, score, count_flag, left_actors, actors_pos, find_time, topic_arrive_time
    actor_id = actor_id_dict[msg.cls]
    for i in actor_id:
        if i not in left_actors:
            continue
        topic_arrive_interval = rospy.get_time() - topic_arrive_time[i]
        topic_arrive_time[i] = rospy.get_time()
        if (msg.x-actors_pos[i].x)**2+(msg.y-actors_pos[i].y)**2<err_threshold**2 and topic_arrive_interval<1:
            if not count_flag[i]:
                count_flag[i] = True
                find_time[i] = rospy.get_time()
                print("find actor_"+str(i))
            elif rospy.get_time() - find_time[i] >= 15:
                del_model('actor_'+str(i))
                left_actors.remove(i)
                print('actor_'+str(i)+' is OK')
                print('Time usage:', time_usage)
                target_finish = 6-len(left_actors)
                # calculate score
                if target_finish == 6:
                    score = (1200 - time_usage) - sensor_cost * 3e-3 
                    print('score:',score)
                    print("Mission finished")
                    while True:
                        pass
                else:
                    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
                    print('score:',score)
                    break        
        else:
            count_flag[i] = False

def actor_info1_callback(msg):
    global target_finish, start_time, score, count_flag, left_actors, actors_pos, find_time, topic_arrive_time,flag_1,flag_2
    red_cnt = 0
    actor_id = actor_id_dict[msg.cls]
    for i in actor_id:
        if i not in left_actors:
            continue
        topic_arrive_interval = rospy.get_time() - topic_arrive_time[i]
        topic_arrive_time[i] = rospy.get_time()
        if (msg.x-actors_pos[i].x)**2+(msg.y-actors_pos[i].y)**2<err_threshold**2 and topic_arrive_interval<1:
            if not count_flag[i]:
                count_flag[i] = True
                find_time[i] = rospy.get_time()
                print("find actor_"+str(i))
                flag_1=i
            elif rospy.get_time() - find_time[i] >= 15:
                del_model('actor_'+str(i))
                left_actors.remove(i)
                print('actor_'+str(i)+' is OK')
                print('Time usage:', time_usage)
                target_finish = 6-len(left_actors)
                # calculate score
                if target_finish == 6:
                    score = (1200 - time_usage) - sensor_cost * 3e-3 
                    print('score:',score)
                    print("Mission finished")
                    while True:
                        pass
                else:
                    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
                    print('score:',score)
                    break        
        else:
            red_cnt+=1
            if red_cnt == 2 and not flag_1==0:
                count_flag[flag_1] = False
                flag_1=0

def actor_info2_callback(msg):
    global target_finish, start_time, score, count_flag, left_actors, actors_pos, find_time, topic_arrive_time,flag_1,flag_2
    red_cnt = 0
    actor_id = actor_id_dict[msg.cls]
    for i in actor_id:
        if i not in left_actors:
            continue
        topic_arrive_interval = rospy.get_time() - topic_arrive_time[i]
        topic_arrive_time[i] = rospy.get_time()
        if (msg.x-actors_pos[i].x)**2+(msg.y-actors_pos[i].y)**2<err_threshold**2 and topic_arrive_interval<1:
            if not count_flag[i]:
                count_flag[i] = True
                find_time[i] = rospy.get_time()
                print("find actor_"+str(i))
                flag_2=i
            elif rospy.get_time() - find_time[i] >= 15:
                del_model('actor_'+str(i))
                left_actors.remove(i)
                print('actor_'+str(i)+' is OK')
                print('Time usage:', time_usage)
                target_finish = 6-len(left_actors)
                 
                # calculate score
                if target_finish == 6:
                    score = (1200 - time_usage) - sensor_cost * 3e-3 
                    print('score:',score)
                    print("Mission finished")
                    while True:
                        pass
                else:
                    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
                    print('score:',score)
                    break        
        else:
            red_cnt+=1
            if red_cnt == 2 and not flag_2==1:
                count_flag[flag_2] = False
                flag_2=1

if __name__ == "__main__":
    left_actors = range(actor_num)
    actors_pos = [None] * actor_num
    count_flag = [False] * actor_num
    topic_arrive_time = [0.0] * actor_num
    find_time = [0.0] * actor_num
    flag_1=0
    flag_2=1
    rospy.init_node('score_cal')
    time.sleep(1)
    start_time = rospy.get_time()
    del_model = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState) 
    score_pub = rospy.Publisher("/score",Int16,queue_size=1) 
    time_usage_pub = rospy.Publisher("/time_usage",Int16,queue_size=1) 
    left_actors_pub = rospy.Publisher("/left_actors",String,queue_size=1)
    actor_blue_sub = rospy.Subscriber("/actor_blue_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_green_sub = rospy.Subscriber("/actor_green_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_white_sub = rospy.Subscriber("/actor_white_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_brown_sub = rospy.Subscriber("/actor_brown_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_red1_sub = rospy.Subscriber("/actor_red1_info",ActorInfo,actor_info1_callback,queue_size=1)
    actor_red2_sub = rospy.Subscriber("/actor_red2_info",ActorInfo,actor_info2_callback,queue_size=1)
    # state_sub = [None] * 6
    # for vehicle_id in range(uav_num):
    #     state_sub[vehicle_id] = rospy.Subscriber('/typhoon_h480_'+str(vehicle_id)+'/mavros/state', State, state_callback, vehicle_id)

    # sensor cost
    mono_cam = 1
    stereo_cam =0
    laser1d = 0
    laser2d = 0
    laser3d = 0
    gimbal = 1
    sensor_cost = mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2+ laser2d * 5e3 + laser3d * 2e4 + gimbal * 2e2
    
    # fall_detect = [0] * 6
    target_finish = 0
    score = (2 + target_finish) * 60 - sensor_cost * 3e-3
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for i in range(6):
            uav_pos_tmp = get_model_state(uav_type+ '_' + str(i), 'ground_plane').pose.position
            if uav_pos_tmp.z > 6.5:
                print("Warning: higher than 6 meter")
                sys.exit(0)
        for i in left_actors:
            actors_pos_tmp = get_model_state('actor_' + str(i), 'ground_plane').pose.position
            if not actors_pos_tmp.x**2+actors_pos_tmp.y**2 == 0:
                actors_pos[i] = actors_pos_tmp
        time_usage = rospy.get_time() - start_time
        if time_usage > 600:
            print('score:',score)
            print("Time out, mission failed")
            while True:
                pass
        score_pub.publish(score)
        time_usage_pub.publish(int(time_usage))
        left_actors_pub.publish(str(left_actors))
        background = cv2.imread("white_background.png")
        cv2.putText(background,"Score: "+str(score) ,(60,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
        cv2.putText(background,"Time usage: "+str(time_usage) ,(360,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
        cv2.putText(background,"Left targets: "+str(left_actors) ,(760,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
        cv2.imshow("Multi-UAV search simulation competition judgment system",background)
        cv2.waitKey( 1)
        rate.sleep()



