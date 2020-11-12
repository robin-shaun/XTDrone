import rospy
import random
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.srv import GetModelState
import sys
import numpy
import copy
from nav_msgs.msg import Odometry
from ObstacleAvoid import ObstacleAviod
import math

class ControlActor:
    def __init__(self, actor_id):
        self.count = 0
        self.shooting_count = 0
        self.uav_num = 6
        self.vehicle_type = 'typhoon_h480'
        self.f = 30
        self.flag = True
        self.distance_flag = True
        self.suitable_point = True
        self.get_moving = False
        self.x = 0.0
        self.y = 0.0
        self.x_max = 150.0
        self.x_min = -50.0
        self.y_max = 50.0
        self.y_min = -50.0
        self.id = actor_id
        self.velocity = 1.5
        self.last_pose = Point()
        self.current_pose = Point()
        self.target_pose = Point()
        self.target_pose.z = 1.25
        # obstacle avoidance:
        self.Obstacleavoid = ObstacleAviod()  #ji wu 
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
        self.black_box = [[[-35.375, -34.825], [-4, -3]], [[-3.275, -2.725], [-4, -3]], [[20.725, 21.275], [-4, -3]], \
        [[57.725, 58.275], [-4, -3]], [[83.725, 84.275], [-4, -3]], [[-25.475, -24.925], [3, 4]], [[8.725, 9.275], [3, 4]], \
        [[26.725, 27.275], [3, 4]], [[70.725, 71.275], [3, 4]], [[90.225, 90.775], [3, 4]],[[53.2, 54.2],[-9.0, -6.72]], \
        [[79, 92], [-33.6, -28]],[[3, 8], [35, 36]], [[-34, -21.7], [18, 33]], [[4.5, 20.5], [11.5, 25]], [[55, 67.3], [15.5, 30.3]], \
        [[71.7, 83.5], [9.5, 18.5]], [[88.3, 100.7], [11.9, 16.2]], [[79, 94.7], [22.2, 34.5]], [[53, 70], [-35, -27]], [[-5.4, 6.6], [-19.4, -10.6]], \
        [[19.0, 38.5], [-22.0, -6.0]], [[-4.7, 4.4], [-33.4, -21.6]], [[-27.8, -23.5], [-27.5, -15]], [[-36, -24], [-35, -31]], \
        [[-35.5, -31.3], [-25.5, -13]], [[3.0,8.2],[34.3,35.7]],[[14,17.5],[-15.7,-12.4]]]
        self.box_num = len(self.black_box)
        self.cmd_pub = rospy.Publisher('/actor_' + str(self.id) + '/cmd_pose', Point, queue_size=10)
        #self.black_box = numpy.array(
        #    [[[-32, -21], [18, 32]], [[7, 18], [12, 26]], [[55, 66], [15, 29]], [[72, 82], [10, 18]],
        #     [[88, 100], [12, 16]], [[79, 94], [24, 33]], [[54, 69], [-34, -27]], [[-4, 4], [-33, -22]],
        #     [[14, 38], [-18, -10]], [[-7, 6], [-19, -11]], [[-27, -24], [-14, -29]], [[-35, -32], [-25, -14]],
        #     [[-36, -24], [-34, -31]]])
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        print('actor_' + self.id + ": " + "communication initialized")
        self.state_uav0_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_0/ground_truth/odom", Odometry, self.cmd_uav0_pose_callback)
        self.state_uav1_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_1/ground_truth/odom", Odometry, self.cmd_uav1_pose_callback)
        self.state_uav2_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_2/ground_truth/odom", Odometry, self.cmd_uav2_pose_callback)
        self.state_uav3_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_3/ground_truth/odom", Odometry, self.cmd_uav3_pose_callback)
        self.state_uav4_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_4/ground_truth/odom", Odometry, self.cmd_uav4_pose_callback)
        self.state_uav5_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+"_5/ground_truth/odom", Odometry, self.cmd_uav5_pose_callback)
    
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
        rospy.init_node('actor_' + str(self.id))
        rate = rospy.Rate(self.f)
        
        while not rospy.is_shutdown():
            self.target_pose.z = 2.0
            self.count = self.count + 1
            # get the pose of uav and actor
            try:
                get_actor_state = self.gazeboModelstate('actor_' + self.id, 'ground_plane')
                self.last_pose = self.current_pose
                self.gazebo_actor_pose = get_actor_state.pose.position
                self.current_pose = self.gazebo_actor_pose
            except rospy.ServiceException as e:
                print("Gazebo model state service"+str(self.id)+"  call failed: %s") % e
                self.current_pose.x = 0.0
                self.current_pose.y = 0.0
                self.current_pose.z = 1.25
            distance = (self.current_pose.x - self.target_pose.x) ** 2 + (
                            self.current_pose.y - self.target_pose.y) ** 2
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
                                print str(self.id) + 'while time :  ', while_time
                                break
                self.target_pose.x = self.x
                self.target_pose.y = self.y
                self.flag = False
                self.distance_flag = False
                self.suitable_point = True
                self.escape_suce_flag = False

                if self.target_pose.x < self.x_min:
                    self.target_pose.x = self.x_min
                elif self.target_pose.x > self.x_max:
                    self.target_pose.x = self.x_max
                if self.target_pose.y < self.y_min:
                    self.target_pose.y = self.y_min
                elif self.target_pose.y > self.y_max:
                    self.target_pose.y = self.y_max
                
                try:
                    print str(self.id)+'general change position'
                    self.subtarget_pos = self.Obstacleavoid.GetPointList(self.current_pose, self.target_pose, 1) # current pose, target pose, safe distance
                    self.subtarget_length = len(self.subtarget_pos)
                    middd_pos = [Point() for k in range(self.subtarget_length)]
                    middd_pos = copy.deepcopy(self.subtarget_pos)
                    self.target_pose.x = copy.deepcopy(middd_pos[0].x)
                    self.target_pose.y = copy.deepcopy(middd_pos[0].y)
                    #self.avoid_start_flag = True
                    # print 'current_position:   ' + str(self.id)+'      ', self.current_pose
                    # print 'middd_pos:     '+ str(self.id)+'      ', middd_pos
                    # print '\n'
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
                                    self.target_pose.x = self.black_box[i][0][1] + 1.0
                                    self.target_pose.y = self.current_pose.y
                                elif dis_min == 1:
                                    self.target_pose.x = self.black_box[i][0][0] - 1.0
                                    self.target_pose.y = self.current_pose.y
                                elif dis_min == 2:
                                    self.target_pose.y = self.black_box[i][1][1] + 1.0
                                    self.target_pose.x = self.current_pose.x
                                else:
                                    self.target_pose.y = self.black_box[i][1][0] - 1.0
                                    self.target_pose.x = self.current_pose.x
                                break
                self.avoid_finish_flag = False

            distance = (self.current_pose.x - self.target_pose.x) ** 2 + (
                            self.current_pose.y - self.target_pose.y) ** 2
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
                print 'angle:   ', angle
                tar_angle = angle - math.pi/2   # escape to the target vertical of the uav
                if tar_angle == math.pi/2:
                    flag_k = 1
                elif (tar_angle == -math.pi/2) or (tar_angle == 3 * math.pi/2):
                    flag_k = 2
                else:
                    k = math.tan(tar_angle) 
                    print 'k:   ', k
                if (tar_angle < 0) and (flag_k == 0):
                    y = k*(self.x_max-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y < self.y_min:
                        self.target_pose.y = self.y_min
                        self.target_pose.x = (self.y_min - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_pose.y = y
                        self.target_pose.x = self.x_max
                elif (tar_angle > 0) and (tar_angle < math.pi/2) and (flag_k == 0):
                    y = k*(self.x_max-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y > self.y_max:
                        self.target_pose.y = self.y_max
                        self.target_pose.x = (self.y_max - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_pose.y = y
                        self.target_pose.x = self.x_max
                elif (tar_angle < math.pi) and (tar_angle > math.pi/2) and (flag_k == 0):
                    y = k*(self.x_min-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y > self.y_max:
                        self.target_pose.y = self.y_max
                        self.target_pose.x = (self.y_max - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_pose.y = y
                        self.target_pose.x = self.x_min
                elif (tar_angle > math.pi) and (tar_angle < 3*math.pi/2) and (flag_k == 0):
                    y = k*(self.x_min-self.gazebo_uav_pose[self.catching_uav_num].x)+self.gazebo_uav_pose[self.catching_uav_num].y
                    if y < self.y_min:
                        self.target_pose.y = self.y_min
                        self.target_pose.x = (self.y_min - self.gazebo_uav_pose[self.catching_uav_num].y) / k + self.gazebo_uav_pose[self.catching_uav_num].x
                    else:
                        self.target_pose.y = y
                        self.target_pose.x = self.x_min
                elif flag_k == 1:
                    self.target_pose.x = self.gazebo_uav_pose[self.catching_uav_num].x
                    self.target_pose.y = self.y_max
                elif flag_k == 2:
                    self.target_pose.x = self.gazebo_uav_pose[self.catching_uav_num].x
                    self.target_pose.y = self.y_min
                print str(self.id) + '   self.curr_pose:', self.gazebo_uav_pose[self.catching_uav_num]
                print str(self.id) + '   self.target_pose:', self.target_pose
                print 'escaping change position'
                try:
                    print str(self.id)+'general change position'
                    self.subtarget_pos = self.Obstacleavoid.GetPointList(self.current_pose, self.target_pose, 1) # current pose, target pose, safe distance
                    self.subtarget_length = 1
                    # middd_pos = [Point() for k in range(self.subtarget_length)]
                    # middd_pos = copy.deepcopy(self.subtarget_pos)
                    self.target_pose.x = self.subtarget_pos[0].x
                    self.target_pose.y = self.subtarget_pos[0].y
                    self.catching_flag = 2
                except:
                    self.target_pose.x = self.target_pose.x
                    self.target_pose.y = self.target_pose.y
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
                    print str(self.id)+ 'i am avoiding'
                    #self.distance_flag = False
                    #self.subtarget_length = len(middd_pos)
                    if self.subtarget_count >= self.subtarget_length:
                        self.avoid_finish_flag = True
                        self.subtarget_count = 0
                    else:
                        self.target_pose.x = middd_pos[self.subtarget_count].x
                        self.target_pose.y = middd_pos[self.subtarget_count].y

            if self.catching_flag == 1 or self.catching_flag == 2:
                self.target_pose.z = 3
            else:
                self.target_pose.z = 2
            if self.count % 200 == 0:
                print str(self.id) + '   vel:', self.target_pose.z
            self.cmd_pub.publish(self.target_pose)
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
