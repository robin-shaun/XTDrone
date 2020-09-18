#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped, TwistStamped
from std_msgs.msg import String 
import sys
import math

class CatchingPlan:

    def __init__(self, uav_id):
        self.uav_type = 'iris'
        self.id = int(uav_id)
        print(self.id)
        self.uav_num = 6
        self.f = 100
        self.count = 0
        self.local_pose = PoseStamped()
        self.uav_current_pose = Point()
        self.uav_current_yaw = 0.0
        self.uav_vel = Twist()
        self.arrive_print = False
        self.flag = 0
        self.Kp = 0.5
        self.Kpy = 0.1
        self.velxy_max = 1
        self.velz_max = 1
        self.angz_max = 0.51
        self.target_position = Twist()
        self.target_yaw = 0.0
        self.arrive_count = 0
        self.local_pose_sub = rospy.Subscriber(self.uav_type + '_' + str(self.id) + "/mavros/local_position/pose",
                                               PoseStamped, self.local_pose_callback)
        self.vel_enu_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd_vel_enu', Twist, queue_size=10)

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.uav_current_pose = self.local_pose.pose.position
        # change Quaternion to TF:
        x = self.local_pose.pose.orientation.x
        y = self.local_pose.pose.orientation.y
        z = self.local_pose.pose.orientation.z
        w = self.local_pose.pose.orientation.w
        r = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        self.uav_current_yaw = r * 180 / math.pi
    def loop(self):
        rospy.init_node('uav'+str(self.id))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            self.count += 1
            if self.flag == 0:
                self.target_position.linear.z = 9.0
                self.target_position.linear.x = self.uav_current_pose.x
                self.target_position.linear.y = self.uav_current_pose.y
                self.uav_vel.angular.x = 0.0
                self.uav_vel.angular.y = 0.0
            if self.flag == 1:
                if self.id == 0:
                    self.target_position.linear.x = -25.0
                    self.target_position.linear.y = 25.0
                elif self.id == 1:
                    self.target_position.linear.x = -25.0
                    self.target_position.linear.y = -25.0
                elif self.id == 2:
                    self.target_position.linear.x = 25.0
                    self.target_position.linear.y = 25.0
                elif self.id == 3:
                    self.target_position.linear.x = 25.0
                    self.target_position.linear.y = -25.0
                elif self.id == 4:
                    self.target_position.linear.x = 75.0
                    self.target_position.linear.y = 25.0
                elif self.id == 5:
                    self.target_position.linear.x = 75.0
                    self.target_position.linear.y = -25.0
                self.flag = 2

            self.uav_vel.linear.x = self.Kp * (self.target_position.linear.x - self.uav_current_pose.x)
            self.uav_vel.linear.y = self.Kp * (self.target_position.linear.y - self.uav_current_pose.y)
            self.uav_vel.linear.z = self.Kp * (self.target_position.linear.z - self.uav_current_pose.z)

            if self.uav_vel.linear.x > self.velxy_max:
                self.uav_vel.linear.x = self.velxy_max
            elif self.uav_vel.linear.x < - self.velxy_max:
                self.uav_vel.linear.x = - self.velxy_max
            if self.uav_vel.linear.y > self.velxy_max:
                self.uav_vel.linear.y = self.velxy_max
            elif self.uav_vel.linear.y < - self.velxy_max:
                self.uav_vel.linear.y = - self.velxy_max
            if self.uav_vel.linear.z > self.velz_max:
                self.uav_vel.linear.z = self.velz_max
            elif self.uav_vel.linear.z < - self.velz_max:
                self.uav_vel.linear.z = - self.velz_max
            if self.uav_vel.linear.x == 0.0:
                if self.uav_vel.linear.y >= 0.0:
                    self.target_yaw = 0.0
                else:
                    self.target_yaw = math.pi
            else:
                self.target_yaw = math.atan2(self.uav_vel.linear.y, self.uav_vel.linear.x)

            self.uav_vel.angular.z = self.Kpy * (self.target_yaw - self.uav_current_yaw)
            if self.uav_vel.angular.z > self.angz_max:
                self.uav_vel.angular.z = self.angz_max
            elif self.uav_vel.angular.z < -self.angz_max:
                self.uav_vel.angular.z = -self.angz_max
            if self.count%20 == 0:
                print('target_position:'+str(self.id),self.target_position)
                print('current_position:'+str(self.id), self.uav_current_pose)
                print('target_yaw:'+str(self.id), self.target_yaw)
                print('uav_current_yaw:'+str(self.id), self.uav_current_yaw)
            self.vel_enu_pub.publish(self.uav_vel)

            if (self.uav_vel.linear.x ** 2 + self.uav_vel.linear.y ** 2 + self.uav_vel.linear.z ** 2) < 0.2:
                self.arrive_count += 1
                if self.arrive_count > 10:
                    self.arrive_print = True
                    self.arrive_count = 0
            else:
                self.arrive_count = 0
                self.arrive_print = False

            if (self.flag == 0) and self.arrive_print:
                self.flag = 1
            elif (self.flag == 2) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x
                self.target_position.linear.y = self.target_position.linear.y - 20.0
                self.flag = 3
            elif (self.flag == 3) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x - 20.0
                self.target_position.linear.y = self.target_position.linear.y
                self.flag = 4
            elif (self.flag == 4) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x
                self.target_position.linear.y = self.target_position.linear.y + 40.0
                self.flag = 5
            elif (self.flag == 5) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x + 40.0
                self.target_position.linear.y = self.target_position.linear.y
                self.flag = 6
            elif (self.flag == 6) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x
                self.target_position.linear.y = self.target_position.linear.y - 30.0
                self.flag = 7
            if (self.flag == 7) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x - 30.0
                self.target_position.linear.y = self.target_position.linear.y
                self.flag = 8
            if (self.flag == 8) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x
                self.target_position.linear.y = self.target_position.linear.y + 20.0
                self.flag = 9
            if (self.flag == 9) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x + 20.0
                self.target_position.linear.y = self.target_position.linear.y
                self.flag = 10
            if (self.flag == 10) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x
                self.target_position.linear.y = self.target_position.linear.y + 10.0
                self.flag = 11
            if (self.flag == 11) and self.arrive_print:
                self.target_position.linear.x = self.target_position.linear.x - 10.0
                self.target_position.linear.y = self.target_position.linear.y
                self.flag = 2
            rate.sleep()



if __name__ == '__main__':
    catchplan = CatchingPlan(sys.argv[1])
    catchplan.loop()