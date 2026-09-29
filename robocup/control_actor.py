#!/usr/bin/python2
# -*- coding:utf-8 -*-
import rospy
import random
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState, SetModelStateRequest, SetModelStateResponse
from std_msgs.msg import String, Time, Float32
import sys
import numpy
import copy
from nav_msgs.msg import Odometry
from ObstacleAvoid import ObstacleAvoid
import math
import ast
import numpy as np
import os



class ControlActor:
    def __init__(self, actor_id):
        self.id = actor_id
        rospy.init_node('actor_' + self.id)
        self.count = 0
        self.shooting_count = 0
        self.uav_num = 6
        self.actor_num = 6
        self.vehicle_type = 'typhoon_h480'
        self.f = 10
        self.spon_dis = 5
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
        self.x_max = 130.0
        self.x_min = -50.0
        self.y_max = 60.0
        self.y_min = -60.0
        self.normal_speed = 1.0
        self.tracked_speed = 2.0
        self.separation_speed = 2.0
        self.actor_avoidance_radius = 4.0
        self.actor_min_distance = 2.0
        self.actor_spawn_distance = 6.0
        self.uav_safety_radius = 7.0
        self.uav_spawn_distance = 8.0
        self.uav_takeoff_points = [(0.0, -3.0), (3.0, -3.0),
                                   (0.0, 0.0), (3.0, 0.0),
                                   (0.0, 3.0), (3.0, 3.0)]
        self.actor_positions = {}
        self.uav_positions = {}
        self.avoid = ActorMotion()
        self.last_pose = Point()
        self.current_pose = Point()
        self.target_motion = Point()
        self.avoid.v = self.normal_speed
        self.teleportation_interval = 25
        self.teleportation_time = Time()
        self.black_box_path = os.path.expanduser('~/XTDrone/robocup/black_box.txt')
        self.black_box = open(self.black_box_path, "r")

        # obstacle avoidance:
        self.Obstacleavoid = ObstacleAvoid()  #ji wu 
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

        # 读取文件并初始化障碍物数据
        with open('obstacle.txt', 'r') as file:
            lines = file.readlines()
            self.obstacle_data = [line.strip().split() for line in lines]
        self.cmd_pub = rospy.Publisher('/actor_' + self.id + '/cmd_motion', ActorMotion, queue_size=10)
        message_pub = rospy.Publisher("/find_actor_%s"%self.id, Float32, queue_size=10)
        
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        print('actor_' + self.id + ": " + "communication initialized")
        self.state_uav0_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_0/ground_truth/odom", Odometry, self.cmd_uav0_pose_callback,queue_size=1)
        self.state_uav1_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_1/ground_truth/odom", Odometry, self.cmd_uav1_pose_callback,queue_size=1)
        self.state_uav2_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_2/ground_truth/odom", Odometry, self.cmd_uav2_pose_callback,queue_size=1)
        self.state_uav3_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_3/ground_truth/odom", Odometry, self.cmd_uav3_pose_callback,queue_size=1)
        self.state_uav4_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_4/ground_truth/odom", Odometry, self.cmd_uav4_pose_callback,queue_size=1)
        self.state_uav5_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_5/ground_truth/odom", Odometry, self.cmd_uav5_pose_callback,queue_size=1)
        self.left_actors_sub = rospy.Subscriber("/left_actors",String,self.left_actors_callback,queue_size=1)
        self.find_actor_sub = rospy.Subscriber("/find_actor_%s"%self.id, Float32, self.actor_teleportation_callback, queue_size=1)
        self.actor_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.actor_states_callback, queue_size=1)
        

    def actor_teleportation_callback(self, msg):
        self.teleportation_time = msg.data
        responce = SetModelStateResponse()
        responce.success = False
        while not responce.success:
            if rospy.get_time() - self.teleportation_time < self.teleportation_interval:
                print(rospy.get_time() - self.teleportation_time)
                continue
            else:
                new_point = SetModelStateRequest()
                new_point.model_state.model_name = "actor_%s"%self.id
                new_point.model_state.pose.position.x, new_point.model_state.pose.position.y = self.create_human_point()
                print(new_point)
                tele = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
                responce = tele(new_point)


    def create_human_point(self):
        count = 1
        spon_dis = 5
        while count < 1e5:
            a = random.uniform(-40, 110)
            b = random.uniform(-40, 40)
            candidate_x = int(a)
            candidate_y = int(b)
            unsafe = False
            
            for box in self.black_box:
                xmin, xmax = box[0]
                ymin, ymax = box[1]
                if ((xmin - spon_dis) < candidate_x < (xmax + spon_dis) and
                        (ymin - spon_dis) < candidate_y < (ymax + spon_dis)):
                    unsafe = True
                    break

            if unsafe:
                count += 1
                continue

            own_name = 'actor_' + self.id
            for actor_name, actor_pose in self.actor_positions.items():
                if actor_name == own_name:
                    continue
                if math.hypot(candidate_x - actor_pose.x,
                              candidate_y - actor_pose.y) < self.actor_spawn_distance:
                    unsafe = True
                    break

            if unsafe:
                count += 1
                continue

            uav_points = list(self.uav_takeoff_points)
            uav_points.extend((pose.x, pose.y) for pose in self.uav_positions.values())
            for uav_x, uav_y in uav_points:
                if math.hypot(candidate_x - uav_x,
                              candidate_y - uav_y) < self.uav_spawn_distance:
                    unsafe = True
                    break

            if not unsafe:
                return candidate_x, candidate_y
            
            count += 1
        
        print('ERROR: Actor generation failed after 100,000 attempts!')
        return None


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

    def actor_states_callback(self, msg):
        actor_positions = {}
        uav_positions = {}
        for index, model_name in enumerate(msg.name):
            if model_name.startswith('actor_'):
                actor_positions[model_name] = msg.pose[index].position
            elif model_name.startswith(self.vehicle_type + '_'):
                uav_positions[model_name] = msg.pose[index].position
        self.actor_positions = actor_positions
        self.uav_positions = uav_positions

    def update_tracking_state(self):
        for i in range(self.uav_num):
            uav_speed_squared = ((self.gazebo_uav_twist[i].x) ** 2 +
                                 (self.gazebo_uav_twist[i].y) ** 2)
            self.dis_actor_uav[i] = ((self.current_pose.x - self.gazebo_uav_pose[i].x) ** 2 +
                                     (self.current_pose.y - self.gazebo_uav_pose[i].y) ** 2) ** 0.5

            if (self.catching_flag == 0 and uav_speed_squared > 1.0 and
                    self.dis_actor_uav[i] < 20.0):
                self.tracking_flag[i] += 1
                if self.tracking_flag[i] > 20:   # tracked for 2 seconds
                    self.catching_flag = 1
                    self.tracking_flag[i] = 0
                    self.catching_uav_num = i
                    print('catch', self.id)
                    break
            else:
                self.tracking_flag[i] = 0

        # Only the UAV that triggered tracking may clear that state. A far
        # unrelated UAV must not cancel another UAV's active pursuit.
        if self.catching_flag != 0 and self.catching_uav_num < self.uav_num:
            catching_distance = ((self.current_pose.x - self.gazebo_uav_pose[self.catching_uav_num].x) ** 2 +
                                 (self.current_pose.y - self.gazebo_uav_pose[self.catching_uav_num].y) ** 2) ** 0.5
            if catching_distance >= 20.0:
                self.catching_flag = 0
                self.catching_uav_num = 10

    def build_motion_command(self):
        """Steer away from nearby actors without changing the planned waypoint."""
        command = ActorMotion()
        command.x = self.avoid.x
        command.y = self.avoid.y
        command.v = self.avoid.v

        desired_x = command.x - self.current_pose.x
        desired_y = command.y - self.current_pose.y
        desired_length = math.sqrt(desired_x ** 2 + desired_y ** 2)
        if desired_length > 1e-6:
            steer_x = desired_x / desired_length
            steer_y = desired_y / desired_length
        else:
            steer_x = 0.0
            steer_y = 0.0

        separation_x = 0.0
        separation_y = 0.0
        nearby_actor = False
        nearby_uav = False
        emergency_separation = False
        own_name = 'actor_' + self.id

        for actor_name, actor_pose in self.actor_positions.items():
            if actor_name == own_name:
                continue

            away_x = self.current_pose.x - actor_pose.x
            away_y = self.current_pose.y - actor_pose.y
            distance = math.sqrt(away_x ** 2 + away_y ** 2)
            if distance >= self.actor_avoidance_radius:
                continue

            nearby_actor = True
            if distance < 1e-6:
                # Give exactly overlapping actors deterministic, different exits.
                angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
                unit_x = math.cos(angle)
                unit_y = math.sin(angle)
            else:
                unit_x = away_x / distance
                unit_y = away_y / distance

            strength = 2.0 * (self.actor_avoidance_radius - distance) / self.actor_avoidance_radius
            steer_x += unit_x * strength
            steer_y += unit_y * strength
            separation_x += unit_x
            separation_y += unit_y
            if distance < self.actor_min_distance:
                emergency_separation = True

        # Keep pedestrians outside the launch / landing footprint even when a
        # UAV is stationary and therefore cannot yet satisfy tracking logic.
        for uav_name, uav_pose in self.uav_positions.items():
            away_x = self.current_pose.x - uav_pose.x
            away_y = self.current_pose.y - uav_pose.y
            distance = math.sqrt(away_x ** 2 + away_y ** 2)
            if distance >= self.uav_safety_radius:
                continue

            nearby_actor = True
            nearby_uav = True
            if distance < 1e-6:
                angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
                unit_x = math.cos(angle)
                unit_y = math.sin(angle)
            else:
                unit_x = away_x / distance
                unit_y = away_y / distance

            # UAV separation takes priority over the current walking target.
            strength = 3.0 * (self.uav_safety_radius - distance) / self.uav_safety_radius
            steer_x += unit_x * strength
            steer_y += unit_y * strength
            separation_x += unit_x * 2.0
            separation_y += unit_y * 2.0
            if distance < self.actor_min_distance:
                emergency_separation = True

        if not nearby_actor:
            return command

        if emergency_separation:
            steer_x = separation_x
            steer_y = separation_y
            command.v = max(command.v, self.separation_speed)

        steer_length = math.sqrt(steer_x ** 2 + steer_y ** 2)
        if steer_length < 1e-6:
            angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
            steer_x = math.cos(angle)
            steer_y = math.sin(angle)
            steer_length = 1.0

        target_distance = (self.uav_safety_radius if nearby_uav
                           else self.actor_avoidance_radius)
        command.x = self.current_pose.x + target_distance * steer_x / steer_length
        command.y = self.current_pose.y + target_distance * steer_y / steer_length
        command.x = max(self.x_min, min(self.x_max, command.x))
        command.y = max(self.y_min, min(self.y_max, command.y))
        return command

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
        rate = rospy.Rate(self.f)
        
        while not rospy.is_shutdown():
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
                    shres = 2
                    self.x = random.uniform(self.x_min-shres, self.x_max+shres)
                    self.y = random.uniform(self.y_min-shres, self.y_max+shres)
                 
                    # 定义阈值
                    collision_threshold = 1.5
                    # 生成路径点
                    path_points = []
                    for t in range(101):  # 生成
                        alpha = t / 100.0
                        x_path = (1 - alpha) * self.current_pose.x+ alpha * self.x
                        y_path = (1 - alpha) * self.current_pose.y + alpha * self.y
                        path_points.append((x_path, y_path))
        
                    for i in range(self.box_num):
                        if (self.x > self.black_box[i][0][0]-self.spon_dis) and (self.x < self.black_box[i][0][1]+self.spon_dis):
                            if (self.y > self.black_box[i][1][0]-self.spon_dis) and (self.y < self.black_box[i][1][1]+self.spon_dis):
                                self.suitable_point = True
                                while_time = while_time+1
                                break

                    for point in path_points:
                        for obstacle in self.obstacle_data :
                            point = [float(point[0]), float(point[1])]
                            obstacle = [float(obstacle[0]), float(obstacle[1])]
                            if np.sqrt((point[0] - obstacle[0])**2 + (point[1] - obstacle[1])**2) < collision_threshold :
                                self.suitable_point = True
                                while_time = while_time+1
                                break
                self.target_motion.x = self.x
                self.target_motion.y = self.y
                self.flag = False
                self.distance_flag = False
                self.suitable_point = True
                self.escape_suce_flag = False

                if self.target_motion.x < self.x_min:
                    self.target_motion.x = self.x_min+5
                elif self.target_motion.x > self.x_max:
                    self.target_motion.x = self.x_max-5
                if self.target_motion.y < self.y_min:
                    self.target_motion.y = self.y_min+5
                elif self.target_motion.y > self.y_max:
                    self.target_motion.y = self.y_max-5              
                try:                   
                    self.subtarget_pos = self.Obstacleavoid.GetPointList(self.current_pose, self.target_motion, 0.5) # current pose, target pose, safe distance
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
                        if (self.current_pose.x > self.black_box[i][0][0]-self.spon_dis) and (self.current_pose.x < self.black_box[i][0][1]+1.0):
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
            self.update_tracking_state()

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
            
            # if self.catching_flag == 1 or self.catching_flag == 2:
            #     self.target_motion.v = 3
            # else:
            #     self.target_motion.v = 2
            #     if self.count % 200 == 0:
            #         print(self.id + '   vel:', self.target_motion.v)
            
            if self.catching_flag in (1, 2):
                self.avoid.v = self.tracked_speed
            else:
                self.avoid.v = self.normal_speed
            # if self.id == 5:
            #     print('self.avoid:', self.avoid)
            self.cmd_pub.publish(self.build_motion_command())
            rate.sleep()


    def random_move(self):
        dis_list = [0,0,0,0]
        for i in range(len(self.black_box)):
            if (self.current_pose.x > self.black_box[i][0][0]-self.spon_dis) and (self.current_pose.x < self.black_box[i][0][1]+1.0):
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
