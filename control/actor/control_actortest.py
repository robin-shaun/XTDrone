import rospy
import random
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.srv import GetModelState
import sys
import numpy
from nav_msgs.msg import Odometry

class ControlActor:
    def __init__(self, actor_id):
        self.count = 0
        self.shooting_count = 0
        self.uav_num = 6
        self.vehicle_type = 'iris'
        self.f = 100
        self.flag = True
        self.distance_flag = False
        self.suitable_point = True
        self.get_moving = False
        self.x = 0.0
        self.y = 0.0
        self.x_max = 100.0
        self.x_min = -50.0
        self.y_max = 50.0
        self.y_min = -50.0
        self.id = actor_id
        self.velocity = 1.5
        self.last_pose = Point()
        self.current_pose = Point()
        self.target_pose = Point()
        self.target_pose.z = 1.25
        self.gazebo_actor_pose = Point()
        self.gazebo_uav_pose = [Point() for i in range(self.uav_num)]
        self.gazebo_uav_twist = [Point()for i in range(self.uav_num)]
        self.dis_actor_uav = [0.0 for i in range(self.uav_num)]   # distance between uav and actor
        self.tracking_flag = [0 for i in range(self.uav_num)]     # check if there is a uav tracking 'me'
        self.catching_flag = 0                                 # if there is a uav tracking 'me' for a long time
        self.catching_uav_num = 10                             # get the number of uav of which is catching 'me'
        self.black_box = numpy.array([[[-34, -19], [16, 34]], [[5, 20], [10, 28]], [[53, 68], [13, 31]], [[70, 84], [8, 20]], [[86, 102], [10, 18]], [[77, 96], [22, 35]], [[52, 71], [-34, -25]], [[-6, 6], [-35, -20]], [[12, 40], [-20, -8]], [[-7, 8], [-21, -9]], [[-29, -22], [-16, -27]], [[-37, -30], [-27, -12]], [[-38, -24], [-36, -29]]])
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
            self.count = self.count + 1
            # get the pose of uav and actor
            try:
                get_actor_state = self.gazeboModelstate('actor_' + self.id, 'ground_plane')
                self.last_pose = self.current_pose
                self.gazebo_actor_pose = get_actor_state.pose.position
                self.current_pose = self.gazebo_actor_pose
            except rospy.ServiceException as e:
                print("Gazebo model state service call failed: %s") % e
                self.current_pose.x = 0.0
                self.current_pose.y = 0.0
                self.current_pose.z = 1.25
            # update new random target position
            if self.flag or self.distance_flag:
                self.x = random.uniform(self.x_min, self.x_max)
                self.y = random.uniform(self.y_min, self.y_max)
                self.flag = False
                self.target_pose.x = self.x
                self.target_pose.y = self.y
                self.distance_flag = False
                self.suitable_point = True
            if self.target_pose.x < self.x_min:
                self.target_pose.x = self.x_min
            elif self.target_pose.x > self.x_max:
                self.target_pose.x = self.x_max
            if self.target_pose.y < self.y_min:
                self.target_pose.y = self.y_min
            elif self.target_pose.y > self.y_max:
                self.target_pose.y = self.y_max
            self.cmd_pub.publish(self.target_pose)
            if self.count % 20 == 0:
                print('current_pose' + self.id+':', self.current_pose)
                print('target_pose' + self.id+':', self.target_pose)
                print('uav_pose:', self.gazebo_uav_pose[0])
                print('uav_vel:', self.gazebo_uav_twist[0])
            distance = (self.current_pose.x-self.target_pose.x)**2+(self.current_pose.y-self.target_pose.y)**2
            if distance < 0.001:
                self.distance_flag = True
                self.catching_flag = 0
            # collosion: if the actor is in the black box, then go backward and update a new target position
            if self.suitable_point:
                for i in range(13):
                    if (self.current_pose.x > self.black_box[i][0][0]) and (self.current_pose.x < self.black_box[i][0][1]):
                        if (self.current_pose.y > self.black_box[i][1][0]) and (self.current_pose.y < self.black_box[i][1][1]):
                            self.target_pose.x = self.current_pose.x + 200*(self.last_pose.x-self.current_pose.x)
                            self.target_pose.y = self.current_pose.y + 200*(self.last_pose.y-self.current_pose.y)
                            self.suitable_point = False
                            self.get_moving = True
                            print('wall!!!!')
                            print('wall!!!!')
                            print('wall!!!!')
                            print('wall!!!!')
                            print('wall!!!!')
                            print('wall!!!!')
                            break
            # dodging uavs: if there is a uav catching 'me', escape
            for i in range(self.uav_num):
                self.dis_actor_uav[i] = (self.current_pose.x-self.gazebo_uav_pose[i].x)**2+(self.current_pose.y-self.gazebo_uav_pose[i].y)**2
                if self.dis_actor_uav[i] < 400.0 and (self.catching_flag == 0):
                    self.tracking_flag[i] = self.tracking_flag[i]+1
                    if self.tracking_flag[i] > 20:
                        self.catching_flag = 1
                        self.tracking_flag[i] = 0
                        self.catching_uav_num = i
                        print('catch', self.id)
                        print('catch', self.id)
                        print('catch', self.id)
                        print('catch', self.id)
                        print('catch', self.id)
                        print('catch', self.id)
                        print('catch', self.id)
                        break
            # escaping
            if self.catching_flag == 1:
                self.catching_flag = 2
                if self.suitable_point:
                    print('escaping')
                    print('escaping')
                    print('escaping')
                    print('escaping')
                    print('escaping')
                    if (self.gazebo_uav_twist[self.catching_uav_num].x >= 0) and (self.gazebo_uav_twist[self.catching_uav_num].y >= 0):
                        self.target_pose.x = self.current_pose.x-10.0*self.gazebo_uav_twist[self.catching_uav_num].x
                        self.target_pose.y = self.current_pose.y+10.0*self.gazebo_uav_twist[self.catching_uav_num].y
                    elif (self.gazebo_uav_twist[self.catching_uav_num].x >= 0) and (self.gazebo_uav_twist[self.catching_uav_num].y < 0):
                        self.target_pose.x = self.current_pose.x+10.0*self.gazebo_uav_twist[self.catching_uav_num].x
                        self.target_pose.y = self.current_pose.y+10.0*self.gazebo_uav_twist[self.catching_uav_num].y
                    elif (self.gazebo_uav_twist[self.catching_uav_num].x < 0) and (self.gazebo_uav_twist[self.catching_uav_num].y >= 0):
                        self.target_pose.x = self.current_pose.x-10.0*self.gazebo_uav_twist[self.catching_uav_num].x
                        self.target_pose.y = self.current_pose.y-10.0*self.gazebo_uav_twist[self.catching_uav_num].x
                    else:
                        self.target_pose.x = self.current_pose.x+10.0*self.gazebo_uav_twist[self.catching_uav_num].x
                        self.target_pose.y = self.current_pose.y-10.0*self.gazebo_uav_twist[self.catching_uav_num].y
                    if self.target_pose == self.current_pose:
                        self.x = random.uniform(self.x_min, self.x_max)
                        self.y = random.uniform(self.y_min, self.y_max)
                        self.target_pose.x = self.x
                        self.target_pose.y = self.y
            # check if the actor is shot:
            if self.get_moving:
                distance_change = (self.last_pose.x - self.current_pose.x)**2 + (self.last_pose.y - self.current_pose.y)**2
                if distance_change < 0.00001:
                    self.shooting_count = self.shooting_count+1
                    if self.shooting_count > 100:
                        print('shot')
                        print('shot')
                        print('shot')
                        print('shot')
                        print('shot')
                        print('shot')
                        self.x = random.uniform(self.x_min, self.x_max)
                        self.y = random.uniform(self.y_min, self.y_max)
                        self.target_pose.x = self.x
                        self.target_pose.y = self.y
                        self.shooting_count = 0
                else:
                    self.shooting_count = 0
            rate.sleep()


if __name__=="__main__":
    controlactors = ControlActor(sys.argv[1])
    controlactors.loop()
