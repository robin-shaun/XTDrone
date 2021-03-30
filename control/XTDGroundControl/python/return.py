import rospy
import PyKDL
from geometry_msgs.msg import Twist, Point, PoseStamped, TwistStamped
from std_msgs.msg import String
import numpy
import math
import sys
import copy
from gazebo_msgs.srv import GetModelState

class ReturnHome:
    def __init__(self,uav_id):
        self.uav_type = 'typhoon_h480'
        self.id = int(uav_id)
        self.uav_num = 6
        self.f = 30  # pin lv
        self.count = 0
        self.local_pose = PoseStamped()
        self.following_local_pose = [PoseStamped() for i in range(self.uav_num)]
        self.following_local_pose_sub = [None]*(self.uav_num)
        self.uav_current_pose = Point()
        self.uav_current_yaw = 0.0
        self.uav_vel = Twist()
        self.arrive_point = False
        self.Kp = 0.5
        self.Kpy = 1
        self.Kpvel = 1
        self.z = 12.0   # height
        self.velxy_max = 4
        self.velz_max = 3
        self.angz_max = 3
        self.bias = [[-10, 32.7],[100,33.5],[75,30],[60,-3],[-30,-40.5],[45,-15]]
        self.target_position = Twist()
        self.target_yaw = 0.0
        self.last_yaw = 0.0
        self.arrive_count = 0
        self.safe_dis = 0.3
        self.safe_height = 0.5
        self.cmd = ''
        self.last_ugv0_pose = Point()
        self.current_ugv0_pose = Point()
        self.last_ugv1_pose = Point()
        self.current_ugv1_pose = Point()
        self.situation_flag = 0
        self.change_task_flag = False
         #variables of rostopic
        rospy.init_node('uav'+str(self.id))
        self.local_pose_sub = rospy.Subscriber(self.uav_type + '_' + str(self.id) + "/mavros/local_position/pose",
                                               PoseStamped, self.local_pose_callback)
        self.vel_enu_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd_vel_flu', Twist, queue_size=10)
        self.cmd_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd',String,queue_size=10)
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        
    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.uav_current_pose = self.local_pose.pose.position
        self.uav_current_pose.x = self.uav_current_pose.x+self.bias[self.id][0]
        self.uav_current_pose.y = self.uav_current_pose.y+self.bias[self.id][1]
        self.uav_current_pose.z = self.uav_current_pose.z
        # change Quaternion to TF:
        x = self.local_pose.pose.orientation.x
        y = self.local_pose.pose.orientation.y
        z = self.local_pose.pose.orientation.z
        w = self.local_pose.pose.orientation.w
        rot = PyKDL.Rotation.Quaternion(x, y, z, w)
        
        res = rot.GetRPY()[2]
        while res > math.pi:
            res -= 2.0*math.pi
        while res < -math.pi:
            res += 2.0*math.pi

        if res < 0:
            res = res + 2.0 * math.pi
        self.uav_current_yaw = res  # 0 to 2pi
        
    def following_local_pose_callback(self, msg, id):
        self.following_local_pose[id] = msg
        self.following_local_pose[id].pose.position.x = self.following_local_pose[id].pose.position.x+self.bias[id][0]
        self.following_local_pose[id].pose.position.y = self.following_local_pose[id].pose.position.y+self.bias[id][1]
        self.following_local_pose[id].pose.position.z = self.following_local_pose[id].pose.position.z
        
    def loop(self):
        rate = rospy.Rate(self.f)
        count_situ_one = 0
        for i in range(self.uav_num):
            if not i == self.id:
                self.following_local_pose_sub[i] = rospy.Subscriber(self.uav_type + '_' + str(i) + "/mavros/local_position/pose", PoseStamped, self.following_local_pose_callback, i)
        while not rospy.is_shutdown():
            self.count += 1
            self.velxy_max = 4.0
            self.cmd = ''
            # get position of ugvs:
            try:
                get_ugv0_state = self.gazeboModelstate('ugv_0', 'ground_plane')
                self.last_ugv0_pose = self.current_ugv0_pose
                self.current_ugv0_pose = get_ugv0_state.pose.position
                
                get_ugv1_state = self.gazeboModelstate('ugv_1', 'ground_plane')
                self.last_ugv1_pose = self.current_ugv1_pose
                self.current_ugv1_pose = get_ugv1_state.pose.position
            except rospy.ServiceException as e:
                print("Gazebo model state service"+"  call failed: %s") % e
            # fly to the same altitude  xian offboard hou arm
            if self.count == 40 or self.count == 42 or self.count == 44:
                self.cmd = 'OFFBOARD'
            if self.count == 94 or self.count == 96 or self.count == 98:
                self.cmd = 'ARM'
            if self.situation_flag == 0:    # ding gao
                self.target_position.linear.z = self.z
                self.target_position.linear.x = self.uav_current_pose.x
                self.target_position.linear.y = self.uav_current_pose.y
                self.uav_vel.angular.x = self.uav_vel.angular.x
                self.uav_vel.angular.y = self.uav_vel.angular.y
                self.uav_vel.angular.z = self.uav_vel.angular.z
                
            if self.situation_flag == 1 and self.change_task_flag:  # chu shi hua
                self.change_task_flag = False
                #if not self.avoid_start_flag:
                self.init_point()
            
            if self.situation_flag == 2 and self.change_task_flag:  # chu shi hua
                self.change_task_flag = False
                #if not self.avoid_start_flag:
                self.return_home()
                print 'flag222222'
                
            distance_tar_cur = self.VectNorm3(self.target_position.linear, self.uav_current_pose)
            if  distance_tar_cur < 0.5:
                self.arrive_count += 1
                if self.arrive_count > 5:
                    self.arrive_point = True
                    self.arrive_count = 0
                else:
                    self.arrive_point = False
            else:
                self.arrive_count = 0
                self.arrive_point = False
                
            # task changes:
            if (self.situation_flag == 0) and self.arrive_point:
                self.change_task_flag = True
                self.situation_flag = 1
                self.arrive_point = False
            elif (self.situation_flag == 1) and self.arrive_point:
                self.situation_flag = 2
                # self.start_yolo_pub.publish('gogogo')
                self.arrive_point = False
                self.change_task_flag = True
                
            if self.situation_flag == 2:
                self.velz_max = 1
                if self.uav_current_pose.z < 2.2:
                    self.target_position.linear.x = self.uav_current_pose.x
                    self.target_position.linear.y = self.uav_current_pose.y
                if self.uav_current_pose.z < 1.9:
                    self.cmd = 'DISARM'
 
            self.get_control_vel()
            # self.obstacle_avoid()
            self.vel_enu_pub.publish(self.uav_vel)
            self.cmd_pub.publish(self.cmd)

            rate.sleep()
            
    def get_control_vel(self): # P kong zhi, flu zuo biao xi
        
        uav_dis_curtar = self.VectNorm2(self.target_position.linear, self.uav_current_pose)   #distance
        temp = self.VectDiff(self.target_position.linear, self.uav_current_pose)  # vector
        uav_vel_total = self.Kp * uav_dis_curtar  # velocity
        if uav_vel_total > self.velxy_max:
            uav_vel_total = self.velxy_max
        '''
        if not uav_dis_curtar == 0.0:
            self.uav_vel.linear.x = (temp.x/uav_dis_curtar) * uav_vel_total
            self.uav_vel.linear.y = (temp.y/uav_dis_curtar) * uav_vel_total
        else:
            self.uav_vel.linear.x = 0.0
            self.uav_vel.linear.y = 0.0


        '''

        self.target_yaw = self.pos2ang(self.target_position.linear.x, self.target_position.linear.y, self.uav_current_pose.x, self.uav_current_pose.y)
        mid_yaw = self.target_yaw - self.uav_current_yaw
        if mid_yaw > math.pi:
            mid_yaw = mid_yaw - 2*math.pi
        elif mid_yaw < -math.pi:
            mid_yaw = 2*math.pi + mid_yaw
        self.uav_vel.angular.z = self.Kpy * mid_yaw
        if self.uav_vel.angular.z > self.angz_max:
            self.uav_vel.angular.z = self.angz_max
        elif self.uav_vel.angular.z < -self.angz_max:
            self.uav_vel.angular.z = -self.angz_max
        self.uav_vel.linear.x  = uav_vel_total * math.cos(mid_yaw)
        self.uav_vel.linear.y = uav_vel_total * math.sin(mid_yaw)

        self.uav_vel.linear.z = self.Kp * (self.target_position.linear.z - self.uav_current_pose.z)
        if self.uav_vel.linear.z > self.velz_max:
            self.uav_vel.linear.z = self.velz_max
        elif self.uav_vel.linear.z < - self.velz_max:
            self.uav_vel.linear.z = - self.velz_max
                       
    def init_point(self):
        if self.id == 0:  # middle circle  3
            self.target_position.linear.x = self.current_ugv0_pose.x   # ugv0 -0.3
            self.target_position.linear.y = self.current_ugv0_pose.y-0.2

        elif self.id == 1:  # middle circle  2
            self.target_position.linear.x = self.current_ugv0_pose.x   # ugv0 0.5
            self.target_position.linear.y = self.current_ugv0_pose.y+0.6

        elif self.id == 2:  # outer loop  0 
            self.target_position.linear.x = self.current_ugv0_pose.x    #ugv0 1.3
            self.target_position.linear.y = self.current_ugv0_pose.y+1.3

        elif self.id == 3:  # outer loop  4
            self.target_position.linear.x = self.current_ugv1_pose.x   #ugv1 -0.3
            self.target_position.linear.y = self.current_ugv1_pose.y-0.2

        elif self.id == 4:  # outer loop  1
            self.target_position.linear.x = self.current_ugv1_pose.x
            self.target_position.linear.y = self.current_ugv1_pose.y+0.6

        elif self.id == 5:  # outer loop  5
            self.target_position.linear.x = self.current_ugv1_pose.x
            self.target_position.linear.y = self.current_ugv1_pose.y+1.3

            
    def return_home(self):
        self.target_position.linear.z = 0.0
            
    # ji jian bi zhang
    def obstacle_avoid(self):
        self.avo_id = []
        for i in range(self.uav_num):
            if not i == self.id:
                dis_partner = math.sqrt(
                    (self.uav_current_pose.x - self.following_local_pose[i].pose.position.x) ** 2 + (self.uav_current_pose.y - self.following_local_pose[i].pose.position.y) ** 2)
                if dis_partner < self.safe_dis:
                    if (self.uav_current_pose.z - self.following_local_pose[i].pose.position.z) < self.safe_height:
                        self.avo_id.append(i)
        avoid_num = len(self.avo_id)
        heigher_num = 0
        if avoid_num > 0:
            for j in range(avoid_num):
                if self.following_local_pose[self.avo_id[j]].pose.position.z > self.uav_current_pose.z:
                    heigher_num = heigher_num + 1
            if heigher_num == 0:
                self.target_position.linear.z = self.target_position.linear.z + self.safe_height
            else:
                self.target_position.linear.z = self.target_position.linear.z - self.safe_height * heigher_num
        else:
            self.target_position.linear.z = self.target_position.linear.z

    def pos2ang(self, xa, ya, xb, yb):   #([xb,yb] to [xa, ya])
        if not xa-xb == 0:
            angle = math.atan2((ya - yb),(xa - xb))
            if (ya-yb > 0) and (angle < 0):
                angle = angle + math.pi
            elif (ya-yb < 0) and (angle > 0):
                angle = angle - math.pi
            elif ya-yb == 0:
                if xa-xb > 0:
                    angle = 0.0
                else:
                    angle = math.pi
        else:
            if ya-yb > 0:
                angle = math.pi / 2
            elif ya-yb <0:
                angle = -math.pi / 2
            else:
                angle = 0.0
        if angle < 0:
            angle = angle + 2 * math.pi   # 0 to 2pi
        return angle

    def VectNorm3(self, Pt1, Pt2):
        norm = math.sqrt(pow(Pt1.x - Pt2.x, 2) + pow(Pt1.y - Pt2.y, 2)+ pow(Pt1.z - Pt2.z, 2))
        return norm

    def VectNorm2(self, Pt1, Pt2):
        norm = math.sqrt(pow(Pt1.x - Pt2.x, 2) + pow(Pt1.y - Pt2.y, 2))
        return norm

    def VectDiff(self, endPt, startPt):
        temp = Point()
        temp.x = endPt.x - startPt.x
        temp.y = endPt.y - startPt.y
        return temp
    
if __name__ == '__main__':
    returnhome = ReturnHome(sys.argv[1])
    returnhome.loop()