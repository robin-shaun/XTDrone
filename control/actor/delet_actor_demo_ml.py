#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteModel,GetModelState

NODE_NAME = 'del_actor'

DAMAGE_RANGE = 15 
MAX_LIFE_VALUE = 20
flag = [False for i in range(6)]
delta_time = [0 for i in range(6)]
judge_time = [0 for i in range(6)]
firsttime = [0 for i in range(6)]
actor_pos = [Point() for i in range(6)]
actor_real_pos = [Point() for i in range(6)]
delta_pose = [0.0 for i in range(6)]
#an uav can cause 2 damge value every second
CHECK_RATE = 1 
exchange_flag = False
judge_flag = [False for i in range(6)]
file = open ('result2.txt','w')
'''
Here to set the vehicle name and number of actor and uav
'''
actor_num = 6
uav_num = 6
VEHICLE_NAME = 'typhoon_h480'


rospy.init_node(NODE_NAME)

rate = rospy.Rate(CHECK_RATE)
count = 0

def string2float(i):
    return float(i)

def judge_callback(msg):
    string_msg = msg.data
    strlist = string_msg.split(' ')
    id = int(strlist[0])
    actor_pos[id].x = string2float(strlist[1])
    actor_pos[id].y = string2float(strlist[2])
    actor_pos[id].z = 0.0
    judge_flag[id] = True
    if not flag[id]:
        firsttime[id] = rospy.get_time()
        flag[id] = True
    else:
        delta_time[id] = rospy.get_time()-firsttime[id]

'''
ros surbcriber
'''
judger_sub = rospy.Subscriber('judger',String,judge_callback)
'''
publisher
'''
human_disappear_pub = rospy.Publisher('judger_react',String,queue_size = 10)

        


'''
ros services
'''
del_model = rospy.ServiceProxy(
    "/gazebo/delete_model",DeleteModel
)

get_model_state = rospy.ServiceProxy(
    "/gazebo/get_model_state",GetModelState
)

life_value_list = [MAX_LIFE_VALUE]*actor_num
def VectNorm2(Pt1, Pt2):
    norm = math.sqrt(pow(Pt1.x - Pt2.x, 2) + pow(Pt1.y - Pt2.y, 2))
    return norm

while not rospy.is_shutdown():
    count += 1
    exchange_flag = False
    for actor_id in range(actor_num):
        try:
            get_actor_state = get_model_state('actor_' + str(actor_id), 'ground_plane')
            actor_real_pos[actor_id] = get_actor_state.pose.position
        except rospy.ServiceException as e:
            print("Gazebo model state service"+str(actor_id)+"  call failed: %s") % e
            actor_real_pos[actor_id] = 0.0
            actor_real_pos[actor_id] = 0.0
            actor_real_pos[actor_id] = 1.25
        if firsttime[actor_id] > 0:
            print 'start !!!!!'
            judge_time[actor_id] = rospy.get_time()-firsttime[actor_id]   
            if judge_flag[actor_id]:
                judge_flag[actor_id] = False
                delta_pose[actor_id] = VectNorm2(actor_real_pos[actor_id],actor_pos[actor_id])
                if actor_id == 0:
                    actor_exchange_id = 5
                    if delta_pose[actor_id] > 10.0:
                        try:
                            get_actor_state = get_model_state('actor_' + str(actor_exchange_id), 'ground_plane')
                            actor_real_pos[actor_exchange_id] = get_actor_state.pose.position
                            delta_pose[actor_exchange_id] = VectNorm2(actor_real_pos[actor_exchange_id],actor_pos[actor_id])
                        except rospy.ServiceException as e:
                            print("Gazebo model state service"+str(actor_exchange_id)+"  call failed: %s") % e
                            actor_real_pos[actor_exchange_id] = 0.0
                            actor_real_pos[actor_exchange_id] = 0.0
                            actor_real_pos[actor_exchange_id] = 1.25
                        if delta_pose[actor_exchange_id] < delta_pose[actor_id]:
                            exchange_flag = True
            if not exchange_flag:
                file.write(str(actor_id)+':')
                file.write('\n')
                file.write(str(actor_real_pos[actor_id]))
                file.write('\n')
                file.write(str(actor_pos[actor_id]))
                file.write('\n')
                file.write(str(delta_pose[actor_id]))
                file.write('\n')
            else:
                file.write(str(actor_exchange_id)+':')
                file.write('\n')
                file.write(str(actor_real_pos[actor_exchange_id]))
                file.write('\n')
                file.write(str(actor_pos[actor_exchange_id]))
                file.write('\n')
                file.write(str(delta_pose[actor_exchange_id]))
                file.write('\n')
            file.write('\n')
        if delta_time[actor_id] >=20:
            if exchange_flag:
                exchange_flag = False
                del_model( 'actor_'+str(actor_exchange_id) )
                print('actor_'+str(actor_exchange_id)+' was killed')
            else:
                del_model( 'actor_'+str(actor_id) )
                print('actor_'+str(actor_id)+' was killed')
            delta_time[actor_id] = 0
            firsttime[actor_id] = 0
            judge_time[actor_id] = 0
            human_disappear_pub.publish(str(actor_id))

        if judge_time[actor_id] > 25:
            delta_time[actor_id] = 0
            firsttime[actor_id] = 0
            judge_time[actor_id] = 0

    #display the life value of all actors
    #for actor_id in range(actor_num):
        #print("actor_"+str(actor_id)+f"::{life_value_list[actor_id]}", end=" ")
    #print(" ")
    rate.sleep()  
file.close()      






        

        
    


