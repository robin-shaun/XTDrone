import rospy
import random
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
import sys
import numpy
import copy
from nav_msgs.msg import Odometry
from ObstacleAvoid import ObstacleAviod
import math
import ast

class ControlActor:
    def __init__(self, actor_id):
        self.count = 0
        self.shooting_count = 0
        self.uav_num = 6
        self.actor_num = 6
        self.vehicle_type = 'typhoon_h480'
        self.f = 10
        self.flag = True
        self.distance_flag = True
        self.suitable_point = True
        self.get_moving = False
        self.x = 0.0
        self.y = 0.0
        # self.x_max = 50.0
        # self.x_min = -10.0
        # self.y_max = -20.0
        # self.y_min = -30.0
        self.x_max = 150.0
        self.x_min = -50.0
        self.y_max = 50.0
        self.y_min = -50.0
        self.id = actor_id
        self.velocity = 1.5
        self.avoid = ActorMotion()
        self.last_pose = Point()
        self.current_pose = Point()
        self.target_motion = Point()
        self.avoid.v = 2
        # obstacle avoidance:
        self.Obstacleavoid = ObstacleAviod()  #ji wu 
        self.left_actors = range(self.actor_num)
        self.avoid_finish_flag = True
        self.subtarget_count = 0
        self.subtarget_length = 0
        self.subtarget_pos = []
        self.arrive_count = 0
        self.escape_suce_flag = False
        self.gazebo_actor_pose = Point()
        self.gazebo_uav_pose = [Point() for i in range(self.uav_num)]
        self.gazebo_uav_twist = [Point()for i in range(self.uav_num)]
        self.dis_actor_uav = [0.0 for i in range(self.uav_num)]   # distance between uav and actor
        self.tracking_flag = [0 for i in range(self.uav_num)]     # check if there is a uav tracking 'me'
        self.catching_flag = 0                               # if there is a uav tracking 'me' for a long time
        self.catching_uav_num = 10                             # get the number of uav of which is catching 'me'
        #self.black_box = numpy.array([[[-34, -19], [16, 34]], [[5, 20], [10, 28]], [[53, 68], [13, 31]], [[70, 84], [8, 20]], [[86, 102], [10, 18]], [[77, 96], [22, 35]], [[52, 71], [-34, -25]], [[-6, 6], [-35, -20]], [[12, 40], [-20, -8]], [[-7, 8], [-21, -9]], [[-29, -22], [-16, -27]], [[-37, -30], [-27, -12]], [[-38, -24], [-36, -29]]])
        content=open("black_box.txt")
        line=content.readline()
        self.black_box=ast.literal_eval(line)
        self.box_num = len(self.black_box)
        self.cmd_pub = rospy.Publisher('/actor_' + self.id + '/cmd_motion', ActorMotion, queue_size=10)
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        print('actor_' + self.id + ": " + "communication initialized")
        self.state_uav0_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_0/ground_truth/odom", Odometry, self.cmd_uav0_pose_callback,queue_size=1)
        self.state_uav1_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_1/ground_truth/odom", Odometry, self.cmd_uav1_pose_callback,queue_size=1)
        self.state_uav2_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_2/ground_truth/odom", Odometry, self.cmd_uav2_pose_callback,queue_size=1)
        self.state_uav3_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_3/ground_truth/odom", Odometry, self.cmd_uav3_pose_callback,queue_size=1)
        self.state_uav4_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_4/ground_truth/odom", Odometry, self.cmd_uav4_pose_callback,queue_size=1)
        self.state_uav5_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_5/ground_truth/odom", Odometry, self.cmd_uav5_pose_callback,queue_size=1)
        self.left_actors_sub = rospy.Subscriber("/left_actors",String,self.left_actors_callback,queue_size=1)

    def left_actors_callback(self, msg):
        left = msg.data
        left = left.replace('[',',')
        left = left.replace(']',',')
        left = left.split(',')
        left_actors = []
        for i in left[1:-1]:
            if i == '':
                continue
            left_actors.append(int(i))
        self.left_actors = left_actors
    def cmd_uav0_pose_callback(self, msg):
        self.gazebo_uav_pose[0] = msg.pose.pose.position
        self.gazebo_uav_twist[0] = msg.twist.twist.linear

    def cmd_uav1_pose_callback(self, msg):
        self.gazebo_uav_pose[1] = msg.pose.pose.position
        self.gazebo_uav_twist[1] = msg.twist.twist.linear

    def cmd_uav2_pose_callback(self, msg):
        self.gazebo_uav_pose[2] = msg.pose.pose.position
        self.gazebo_uav_twist[2] = msg.twist.twist.linear

    def cmd_uav3_pose_callback(self, msg):
        self.gazebo_uav_pose[3] = msg.pose.pose.position
        self.gazebo_uav_twist[3] = msg.twist.twist.linear

    def cmd_uav4_pose_callback(self, msg):
        self.gazebo_uav_pose[4] = msg.pose.pose.position
        self.gazebo_uav_twist[4] = msg.twist.twist.linear

    def cmd_uav5_pose_callback(self, msg):
        self.gazebo_uav_pose[5] = msg.pose.pose.position
        self.gazebo_uav_twist[5] = msg.twist.twist.linear

    def loop(self):
        rospy.init_node('actor_' + self.id)
        rate = rospy.Rate(self.f)
        
        while not rospy.is_shutdown():
            self.avoid.v = 2.0
            self.count = self.count + 1
            # get the pose of uav and actor
            if not int(self.id) in self.left_actors:
                print('actor_' + self.id + ' has been deleted')
                break
            try:
                get_actor_state = self.gazeboModelstate('actor_' + self.id, 'ground_plane')
                self.last_pose = self.current_pose
                self.gazebo_actor_pose = get_actor_state.pose.position
                self.current_pose = self.gazebo_actor_pose
            except rospy.ServiceException as e:
                print("Gazebo model state service"+self.id+"  call failed: %s") % e
                self.current_pose.x = 0.0
                self.current_pose.y = 0.0
                self.current_pose.z = 1.25
                # collosion: if the actor is in the black box, then go backward and update a new target position
            # update new random target position
            if (self.avoid_finish_flag and self.distance_flag):
                while self.suitable_point:
                    while_time = 0
                    self.suitable_point = False
                    self.x = random.uniform(self.x_min, self.x_max)
                    self.y = random.uniform(self.y_min, self.y_max)
                    for i in range(self.box_num):
                        if (self.x > self.black_box[i][0][0]-1.5) and (self.x < self.black_box[i][0][1]+1.5):
                            if (self.y > self.black_box[i][1][0]-1.5) and (self.y < self.black_box[i][1][1]+1.5):
                                self.suitable_point = True
                                while_time = while_time+1
                                #print self.id + 'while time :  ', while_time
                                break
                self.target_motion.x = self.x
                self.target_motion.y = self.y
                self.flag = False
                self.distance_flag = False
                self.suitable_point = True
                self.escape_suce_flag = False

                if self.target_motion.x < self.x_min:
                    self.target_motion.x = self.x_min
                elif self.target_motion.x > self.x_max:
                    self.target_motion.x = self.x_max
                if self.target_motion.y < self.y_min:
                    self.target_motion.y = self.y_min
                elif self.target_motion.y > self.y_max:
                    self.target_motion.y = self.y_max               
                try:                   
                    self.subtarget_pos = self.Obstacleavoid.GetPointList(self.current_pose, self.target_motion, 1) # current pose, target pose, safe distance
                    print(self.subtarget_pos)
                    self.subtarget_length = len(self.subtarget_pos)
                    middd_pos = [Point() for k in range(self.subtarget_length)]
                    middd_pos = copy.deepcopy(self.subtarget_pos)
                    self.avoid.x = copy.deepcopy(middd_pos[0].x)
                    self.avoid.y = copy.deepcopy(middd_pos[0].y)
                    #self.avoid_start_flag = True
                    if (self.id == '5' or self.id == '4'):
                        print(self.id+' general change position')
                        print('current_position:   ' + self.id+'      ', self.current_pose)
                        print('middd_pos:     '+ self.id+'      ', middd_pos)
                        print('\n')
                except:
                    dis_list = [0,0,0,0]
                    for i in range(len(self.black_box)):
                        if (self.current_pose.x > self.black_box[i][0][0]-1.0) and (self.current_pose.x < self.black_box[i][0][1]+1.0):
                            if (self.current_pose.y > self.black_box[i][1][0]-1.0) and (self.current_pose.y < self.black_box[i][1][1]+1.0):
                                dis_list[0] = abs(self.current_pose.x - (self.black_box[i][0][0] - 1.0))
                                dis_list[1] = abs(self.current_pose.x - (self.black_box[i][0][1] + 1.0))
                                dis_list[2] = abs(self.current_pose.y - (self.black_box[i][1][0] - 1.0))
                                dis_list[3] = abs(self.current_pose.y - (self.black_box[i][1][1] + 1.0))
                                dis_min = dis_list.index(min(dis_list))
                                self.subtarget_length = 1
                                if dis_min == 0:
                                    self.avoid.x = self.black_box[i][0][0] - 1.0
                                    self.avoid.y = self.current_pose.y
                                elif dis_min == 1:
                                    self.avoid.x = self.black_box[i][0][1] + 1.0
                                    self.avoid.y = self.current_pose.y
                                elif dis_min == 2:
                                    
                                    self.avoid.y = self.black_box[i][1][0] - 1.0
                                    self.avoid.x = self.current_pose.x
                                else:
                                    self.avoid.y = self.black_box[i][1][1] + 1.0
                                    self.avoid.x = self.current_pose.x
                                break
                    if(self.id == '5' or self.id == '4'):
                        print(self.id+'change position except')
                        print('current_position:   ' + self.id+'      ', self.current_pose)
                        print('target_motion:     '+ self.id+'      ', self.target_motion)
                        print('\n')
                self.avoid_finish_flag = False

            distance = (self.current_pose.x - self.avoid.x) ** 2 + (
                            self.current_pose.y - self.avoid.y) ** 2
            if distance < 0.01:
                self.arrive_count += 1
                if self.arrive_count > 5:
                    self.distance_flag = True
                    self.arrive_count = 0
                    if self.catching_flag == 2:
                        self.catching_flag = 0
                else:
                    self.distance_flag = False
            else:
                self.arrive_count = 0
                self.distance_flag = False            

            # dodging uavs: if there is a uav catching 'me', escape
            for i in range(self.uav_num):
                if ((self.gazebo_uav_twist[i].x)**2+(self.gazebo_uav_twist[i].y)**2) > 1.0:
                    self.dis_actor_uav[i] = ((self.current_pose.x-self.gazebo_uav_pose[i].x)**2+(self.current_pose.y-self.gazebo_uav_pose[i].y)**2)**0.5
                    if self.dis_actor_uav[i] < 20.0 and (self.catching_flag == 0):
                        self.tracking_flag[i] = self.tracking_flag[i]+1
                        if self.tracking_flag[i] > 20:   # 2s and excape
                            self.catching_flag = 1
                            self.tracking_flag[i] = 0
                            self.catching_uav_num = i
                            print('catch', self.id)
                            print('catch', self.id)
                            break  
                    if self.dis_actor_uav[i] >= 20.0:
                        # if self.catching_flag[i] == 1 or self.catching_flag[i] == 2:
                        #     self.escape_suce_flag = True
                        self.tracking_flag[i] = 0
                        self.catching_flag = 0
                        self.catching_uav_num = 10

            # # escaping (get a new target position)
            if self.catching_flag == 1:
                flag_k = 0
                angle = self.pos2ang(self.gazebo_uav_twist[self.catching_uav_num].x,self.gazebo_uav_twist[self.catching_uav_num].y)
                print('angle:   ', angle)
                tar_angle = angle - math.pi/2   # escape to the target vertical of the uav
                if tar_angle == math.pi/2:
                    flag_k = 1
                elif (tar_angle == -math.pi/2) or (tar_angle == 3 * math.pi/2):
                    flag_k = 2
                else:
                    k = math.tan(tar_angle) 
                    print('k:   ', k)
                if (tar_angle < 0) and (flag_k == 0):
                    y = k*(self.x_max-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y < self.y_min:
                        self.target_motion.y = self.y_min
                        self.target_motion.x = (self.y_min - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_motion.y = y
                        self.target_motion.x = self.x_max
                elif (tar_angle > 0) and (tar_angle < math.pi/2) and (flag_k == 0):
                    y = k*(self.x_max-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y > self.y_max:
                        self.target_motion.y = self.y_max
                        self.target_motion.x = (self.y_max - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_motion.y = y
                        self.target_motion.x = self.x_max
                elif (tar_angle < math.pi) and (tar_angle > math.pi/2) and (flag_k == 0):
                    y = k*(self.x_min-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y > self.y_max:
                        self.target_motion.y = self.y_max
                        self.target_motion.x = (self.y_max - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_motion.y = y
                        self.target_motion.x = self.x_min
                elif (tar_angle > math.pi) and (tar_angle < 3*math.pi/2) and (flag_k == 0):
                    y = k*(self.x_min-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y < self.y_min:
                        self.target_motion.y = self.y_min
                        self.target_motion.x = (self.y_min - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_motion.y = y
                        self.target_motion.x = self.x_min
                elif flag_k == 1:
                    self.target_motion.x = self.gazebo_uav_pose[self.catching_uav_num].x
                    self.target_motion.y = self.y_max
                elif flag_k == 2:
                    self.target_motion.x = self.gazebo_uav_pose[self.catching_uav_num].x
                    self.target_motion.y = self.y_min
                if self.id == 5:
                    print(self.id + '   self.curr_pose:', self.gazebo_uav_pose[self.catching_uav_num])
                    print(self.id + '   self.target_motion:', self.target_motion)
                    print('escaping change position')
                try:
                    print(self.id+'general change position')
                    self.subtarget_pos = self.Obstacleavoid.GetPointList(self.current_pose, self.target_motion, 1) # current pose, target pose, safe distance
                    self.subtarget_length = 1
                    # middd_pos = [Point() for k in range(self.subtarget_length)]
                    # middd_pos = copy.deepcopy(self.subtarget_pos)
                    self.avoid.x = self.subtarget_pos[0].x
                    self.avoid.y = self.subtarget_pos[0].y
                    self.catching_flag = 2
                except:
                    self.avoid.x = self.target_motion.x
                    self.avoid.y = self.target_motion.y
                    self.catching_flag = 1


            # check if the actor is shot:
            if self.get_moving:
                distance_change = (self.last_pose.x - self.current_pose.x)**2 + (self.last_pose.y - self.current_pose.y)**2
                if distance_change < 0.00001:
                    self.shooting_count = self.shooting_count+1
                    if self.shooting_count > 1000:
                        print('shot', self.id)
                        print('shot', self.id)
                        self.distance_flag = False
                        self.suitable_point = True
                        self.avoid_finish_flag = True
                        self.shooting_count = 0
                else:
                    self.shooting_count = 0

            

            if not self.avoid_finish_flag:
                if self.distance_flag:
                    self.subtarget_count += 1
                    self.distance_flag = False
                    #print self.id+ ': I am avoiding'
                    #self.distance_flag = False
                    #self.subtarget_length = len(middd_pos)
                    if self.subtarget_count >= self.subtarget_length:
                        self.avoid_finish_flag = True
                        self.subtarget_count = 0
                    else:
                        self.avoid.x = middd_pos[self.subtarget_count].x
                        self.avoid.y = middd_pos[self.subtarget_count].y
            '''
            if self.catching_flag == 1 or self.catching_flag == 2:
                self.target_motion.v = 3
            else:
                self.target_motion.v = 2
                if self.count % 200 == 0:
                print self.id + '   vel:', self.target_motion.v
            '''
            #reduce difficulty
            self.avoid.v = 1
            if self.id == 5:
                print('self.avoid:', self.avoid)
            self.cmd_pub.publish(self.avoid)
            rate.sleep()

    def pos2ang(self, deltax, deltay):   #([xb,yb] to [xa, ya])
        if not deltax == 0:
            angle = math.atan2(deltay,deltax)
            if (deltay > 0) and (angle < 0):
                angle = angle + math.pi
            elif (deltay < 0) and (angle > 0):
                angle = angle - math.pi
            elif deltay == 0:
                if deltax > 0:
                    angle = 0.0
                else:
                    angle = math.pi
        else:
            if deltay > 0:
                angle = math.pi / 2
            elif deltay <0:
                angle = -math.pi / 2
            else:
                angle = 0.0
        if angle < 0:
            angle = angle + 2 * math.pi   # 0 to 2pi
        return angle


if __name__=="__main__":
    controlactors = ControlActor(sys.argv[1])
    controlactors.loop()
