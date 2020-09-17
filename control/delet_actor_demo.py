#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.srv import DeleteModel,GetModelState

NODE_NAME = 'del_actor'

DAMAGE_RANGE = 2 
MAX_LIFE_VALUE = 20

#an uav can cause 2 damge value every second
CHECK_RATE = 2 

'''
Here to set the vehicle name and number of actor and uav
'''
actor_num = 2
uav_num = 3
VEHICLE_NAME = 'iris'


rospy.init_node(NODE_NAME)

rate = rospy.Rate(CHECK_RATE)


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

while not rospy.is_shutdown():
    for actor_id in range(actor_num):
        if life_value_list[actor_id]>0:   

            damage_value = 0

            for uav_id in range(uav_num):
                # get the position vector from uav to actor
                p = get_model_state('actor_'+str(actor_id),VEHICLE_NAME+'_'+str(uav_id)).pose.position
                # calculate the distance
                distance = ( (p.x)**2 +(p.y)**2 + (p.z)**2 )**0.5

                if distance <= DAMAGE_RANGE:
                    damage_value += 1
   
            if damage_value == 0: 
            #If no damage casused in a check frame, 
            #the actor's life value will go back to MAX_LIFE_VALUE.
                life_value_list[actor_id] = MAX_LIFE_VALUE
            else:
                life_value_list[actor_id] -= damage_value            

            if life_value_list[actor_id] <= 0:
                del_model( 'actor_'+str(actor_id) )
                print('actor_'+str(actor_id)+' was killed')

    #display the life value of all actors
    for actor_id in range(actor_num):
        print("actor_"+str(actor_id)+f"::{life_value_list[actor_id]}",end=" ")
    print(" ")
    rate.sleep()        






        

        
    


