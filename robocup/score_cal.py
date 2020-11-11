import rospy
import math
from geometry_msgs.msg import Point
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteModel,GetModelState

NODE_NAME = 'score_cal'

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
actor_finish = []
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

def judge_callback(msg):
    string_msg = msg.data
    strlist = string_msg.split(' ')
    id = int(strlist[0])
    actor_pos[id].x = float(strlist[1])
    actor_pos[id].y = float(strlist[2])
    actor_pos[id].z = 0.0
    exchange_flag = False
    if id == 0:
        if not id in actor_finish:
            delta_pose[id] = VectNorm2(actor_real_pos[id],actor_pos[id])
        else:
            exchange_flag = True
        if delta_pose[id] > 20.0 or exchange_flag:
            if not 5 in actor_finish:
                delta_pose[5] = VectNorm2(actor_real_pos[5],actor_pos[id])
                if delta_pose[5] < delta_pose[id]:
                    id = 5 
                    actor_pos[id].x = float(strlist[1])
                    actor_pos[id].y = float(strlist[2])
                    actor_pos[id].z = 0.0                    
    judge_flag[id] = True
    if not flag[id]:
        firsttime[id] = rospy.get_time()
        flag[id] = True
    else:
        delta_time[id] = rospy.get_time()-firsttime[id]

if __name__ == "__main__":
    judger_sub = rospy.Subscriber('judger',String,judge_callback)
    human_disappear_pub = rospy.Publisher('judger_react',String,queue_size = 10)
    del_model = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)

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
                if not actor_id in actor_finish:
                    actor_finish.append(actor_id)
            if firsttime[actor_id] > 0:
                print 'start !!!!!'
                judge_time[actor_id] = rospy.get_time()-firsttime[actor_id]   
                if judge_flag[actor_id]:
                    judge_flag[actor_id] = False
                    delta_pose[actor_id] = VectNorm2(actor_real_pos[actor_id],actor_pos[actor_id])
                file.write(str(actor_id)+':')
                file.write('\n')
                file.write(str(actor_real_pos[actor_id]))
                file.write('\n')
                file.write(str(actor_pos[actor_id]))
                file.write('\n')
                file.write(str(delta_pose[actor_id]))
                file.write('\n')
                file.write('\n')
            if delta_time[actor_id] >=20:
                if not actor_id in actor_finish: 
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
                flag[actor_id] = False

        #display the life value of all actors
        #for actor_id in range(actor_num):
            #print("actor_"+str(actor_id)+f"::{life_value_list[actor_id]}", end=" ")
        #print(" ")
        rate.sleep()  
    file.close()      

    # 传感器费用
    mono_cam = 1
    stereo_cam =0
    laser1d = 0
    laser2d = 0
    laser3d = 0
    gimbal = 1
    sensor_cost = mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2+ laser2d * 5e3 + laser3d * 2e4 + gimbal * 2e2

    # 计算得分
    if success:
        score = (800 - time_usage) - sensor_cost * 3e-3
    else:
        score = (1 + target_finish) * 2e2 - sensor_cost * 3e-3

