# -*- coding: UTF-8 -*-
"""
Framework for Formation Control

* To control all UAV in Gazebo(simulation environment) using MPI

Before running this code, you need to launch your simulation Environment and px4 via:
$ roslaunch px4 cic2021.launch    # 3 UAVs in simulation

And then, you can run this code via:
$ mpiexec -n 3 python formation_sim.py            # 3 UAVs in simulation
"""

import time
from pyquaternion import Quaternion
from mpi4py import MPI
import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, SetMavFrame
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import numpy as np
import copy
from fuzzy_pid import FuzzyPID


uav_type = 'typhoon_h480'
comm = MPI.COMM_WORLD
uav_id = comm.Get_rank()
uav_num = comm.Get_size()

uav_bias = [[0,0,0],[0,3,0],[0,-3,0]]

class Px4Controller:
    def __init__(self):
        self.uav_id = uav_id
        self.namespace = uav_type + '_' + "{proc}".format(proc=self.uav_id)

        self.imu = Imu()
        self.gps = NavSatFix()
        self.local_pose = PoseStamped()

        self.neighbor_id = []

        self.current_heading = None
        self.takeoff_height = 6

        self.cur_target_pose = None
        self.target_yaw = 0

        self.takeoff_target_pose = PoseStamped()
        self.hover_target_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_vel = TwistStamped()
        self.global_pose = [PoseStamped() for i in range(uav_num)]

        self.received_new_task = False
        self.arm_state = False
        self.mavros_state = State()

        self.gcs_cmd = String()
        self.last_gcs_cmd = String()
        self.form_flag = 0

        # control parameters
        self.fuzzy_pid = FuzzyPID()
        self.Kpx = 1
        self.Kpy = 1
        self.Kpz = 1
        self.Kpw = 1
        self.velxy_max = 1
        self.velz_max = 1
        self.w_max = 0.5

        # ros subscribers
        for i in range(uav_num):
            rospy.Subscriber(uav_type+ '_' + str(i) + "/mavros/local_position/pose", PoseStamped, self.local_pose_callback, i, queue_size = 2)
        # self.local_vel_sub = rospy.Subscriber(uav_type + '_' + str(i) + "/mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        self.mavros_sub = rospy.Subscriber(self.namespace + "/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber(self.namespace + "/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber(self.namespace + "/mavros/imu/data", Imu, self.imu_callback)
        self.gcs_cmd_sub = rospy.Subscriber("/formation_gcs/cmd", String, self.gcs_cmd_callback)

        # ros publishers
        self.local_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.twist_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        # ros services
        self.armService = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        self.frameService = rospy.ServiceProxy(self.namespace + '/mavros/setpoint_velocity/mav_frame', SetMavFrame)
        print(self.namespace, ": Px4 Controller Start!")

    def working(self):
        rospy.init_node(self.namespace + "_control_node")
        time.sleep(1) #waiting or node init 1s
        rate = rospy.Rate(60)

        for i in range(10):
            if self.current_heading is not None:
                print(self.namespace, ": initialization finished !")
                break
            else:
                print(self.namespace, ": Waiting for initialization...")
                time.sleep(0.5)

        while rospy.is_shutdown() is False:

            if self.gcs_cmd == 'TAKEOFF':
                self.last_gcs_cmd = 'TAKEOFF'
                self.form_flag = 0
                if self.mavros_state != 'OFFBOARD':
                    self.flight_mode_set(mode='OFFBOARD')
                if not self.arm_state:
                    self.arm()

                self.target_pose = self.construct_target(self.takeoff_target_pose.pose.position.x,
                                                         self.takeoff_target_pose.pose.position.y , self.takeoff_height,
                                                         self.current_heading)
                self.local_target_pub.publish(self.target_pose)

                if self.takeoff_detection():
                    print(self.namespace, ": Takeoff Success !")
                else:
                    print(self.namespace, ": Takeoff Failed !!!")
                # self.frameService(1)

            elif self.gcs_cmd == 'FORM_0':
                self.last_gcs_cmd = 'FORM_0'
                if self.mavros_state != 'OFFBOARD':
                    self.flight_mode_set(mode='OFFBOARD')

            elif self.gcs_cmd == 'FORM_1':
                self.last_gcs_cmd = 'FORM_1'
                if self.form_flag != 1:
                    self.read_set_file('FORM_1_id','FORM_1_pos')
                self.form_flag = 1
                if self.mavros_state != 'OFFBOARD':
                    self.flight_mode_set(mode='OFFBOARD')
                self.formation_control()
                self.twist_target_pub.publish(self.target_vel)

            elif self.gcs_cmd == 'FORM_2':
                self.last_gcs_cmd = 'FORM_2'
                if self.form_flag != 2:
                    self.read_set_file('FORM_2_id','FORM_2_pos')
                self.form_flag = 2
                if self.mavros_state != 'OFFBOARD':
                    self.flight_mode_set(mode='OFFBOARD')
                self.formation_control()
                self.twist_target_pub.publish(self.target_vel)

            elif self.gcs_cmd == 'AUTO.LAND':
                self.last_gcs_cmd = 'AUTO.LAND'
                self.form_flag = 0
                if self.mavros_state != "AUTO.LAND":
                    self.flight_mode_set(mode='AUTO.LAND')
                if (self.mavros_state == 'AUTO.LAND') and (self.local_pose.pose.position.z < 0.05):
                    if self.arm_state:
                        self.disarm()
                    print(self.namespace, ": Land Success!")
            elif self.gcs_cmd == 'HOVER':
                self.last_gcs_cmd = 'HOVER'
                self.form_flag = 0
                if self.mavros_state != 'OFFBOARD':
                    self.flight_mode_set(mode='OFFBOARD')
                if not self.arm_state:
                    self.arm()

                self.target_pose = self.construct_target(self.hover_target_pose.pose.position.x,
                                                         self.hover_target_pose.pose.position.y, self.hover_target_pose.pose.position.z,
                                                         self.current_heading)
                self.local_target_pub.publish(self.target_pose)
            else:
                self.gcs_cmd = self.last_gcs_cmd
                self.form_flag = 0

            rate.sleep()

    def formation_control(self):
        print('formation control here')
        neighbor_num = len(self.neighbor_id)
        self.target_vel.twist.linear.x = 0
        self.target_vel.twist.linear.y = 0
        self.target_vel.twist.linear.z = 0
        self.target_vel.twist.angular.x = 0
        self.target_vel.twist.angular.y = 0
        self.target_vel.twist.angular.z = self.Kpw*(self.target_yaw-self.current_heading)
        # print("neighbor_num",neighbor_num)
        for i in range(neighbor_num):
            self.target_vel.twist.linear.x += self.global_pose[self.neighbor_id[i]].pose.position.x - self.global_pose[self.uav_id].pose.position.x - \
                                                self.all_desired_position[self.neighbor_id[i]][0] + self.all_desired_position[self.uav_id][0]
            self.target_vel.twist.linear.y += self.global_pose[self.neighbor_id[i]].pose.position.y - self.global_pose[self.uav_id].pose.position.y - \
                                                self.all_desired_position[self.neighbor_id[i]][1] + self.all_desired_position[self.uav_id][1]
            self.target_vel.twist.linear.z += self.global_pose[self.neighbor_id[i]].pose.position.z - self.global_pose[self.uav_id].pose.position.z - \
                                                self.all_desired_position[self.neighbor_id[i]][2] + self.all_desired_position[self.uav_id][2]

        self.target_vel.twist.linear.x = self.limit(self.target_vel.twist.linear.x * self.Kpx, -self.velxy_max, self.velxy_max)
        self.target_vel.twist.linear.y = self.limit(self.target_vel.twist.linear.y * self.Kpy, -self.velxy_max, self.velxy_max)
        self.target_vel.twist.linear.z = self.limit(self.target_vel.twist.linear.z * self.Kpz, -self.velz_max, self.velz_max)
        self.target_vel.twist.angular.z = self.limit(self.target_vel.twist.angular.z * self.Kpw, -self.w_max, self.w_max)

    def limit(self, data, min, max):
        if data <= min:
            data = min
        elif data >= max:
            data = max
        return data

    def read_set_file(self,txt_id,txt_pos):
        self.neighbor_id =[]
        id_path='txt/'+txt_id+'.txt'
        pos_path='txt/'+txt_pos+'.txt'
        txt_uav_neighbor_num = np.loadtxt(id_path,dtype=int)
        self.all_desired_position = np.loadtxt(pos_path)
        for i in range(0, len(txt_uav_neighbor_num[:, 0])):
                if txt_uav_neighbor_num[i, 0] == self.uav_id:
                   self.neighbor_id.append(txt_uav_neighbor_num[i, 1])
        print(self.neighbor_id)

    def construct_target(self, x, y, z, yaw):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 7

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z
        target_raw_pose.yaw = yaw

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE \
                                    + PositionTarget.FORCE
        return target_raw_pose

    '''
    Callback Function
    '''
    def gcs_cmd_callback(self, msg):
        self.gcs_cmd = msg.data

    def local_pose_callback(self, msg, i):
        if i == uav_id:
            self.local_pose = copy.deepcopy(msg)
        if self.gcs_cmd != 'TAKEOFF':
            self.takeoff_target_pose = copy.deepcopy(self.local_pose)
        if self.gcs_cmd !='HOVER':
            self.hover_target_pose = copy.deepcopy(self.local_pose)
        self.global_pose[i] = copy.deepcopy(msg)
        self.global_pose[i].pose.position.x += uav_bias[i][0]
        self.global_pose[i].pose.position.y += uav_bias[i][1]
        self.global_pose[i].pose.position.z += uav_bias[i][2]
    #
    # def local_vel_callback(self):
    #
    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        self.arm_state = msg.armed

    def imu_callback(self, msg):
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation)

    def gps_callback(self, msg):
        self.gps = msg

    '''
    return yaw from current IMU
    '''
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def flight_mode_set(self, mode):
        """ mode selectable
        MANUAL, ACRO, ALTCTL, POSCTL, OFFBOARD, STABILIZED, RATTITUDE
        AUTO.MISSION, AUTO.LOITER, AUTO.RTL, AUTO.LAND, AUTO.RTGS, AUTO.READY, AUTO.TAKEOFF
        """
        if self.flightModeService(custom_mode=mode):
            return True
        else:
            print(self.namespace + mode + "Failed")

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.2 and self.arm_state:
            return True
        else:
            return False

if __name__ == '__main__':
    px4controller = Px4Controller()
    px4controller.working()
