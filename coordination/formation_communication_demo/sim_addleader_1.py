# -*- coding: UTF-8 -*-
"""
* This is the version that leader can hover, last modify on 26th May by Lan

Framework for Parallel Simulation

* To control all UAV in Gazebo(simulation environment) using MPI

Before running this code, you need to launch your simulation Environment and px4 via:
$ roslaunch px4 iris3_parallel_simulation.launch    # 3 UAVs in simulation
$ roslaunch px4 iris10_parallel_simulation.launch   # 10 UAVs in simulation

And then, you can run this code via:
$ mpiexec -n 3 python px4_control_sim_addleader.py            # 3 UAVs in simulation
$ mpiexec -n 10 python px4_control_sim_addleader.py           # 10 UAVs in simulation
"""+

import tf
import time
import math
from pyquaternion import Quaternion
from mpi4py import MPI
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetMavFrame
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
from control.msg import UAVDataMsg, AllUAVData, NeighborMsg, AllNeighborMsg, CommVerify, UavComm
import numpy as np
import copy
import tf_conversions as tfc

comm = MPI.COMM_WORLD
uav_id = comm.Get_rank()
uav_num_sim = comm.Get_size()
uav_num_real = 0
uav_num = uav_num_sim + uav_num_real0
uav_bias = [[0,0,0],[1.2,-1.2,0],[-1.2,-1.2,0],[1.2,1.2,0],[-1.2,1.2,0],[2.4,-2.4,0],[0,-2.4,0],[-2.4,-2.4,0],[2.4,0,0],[-2.4,0,0],[2.4,2.4,0],[0,2.4,0],[-2.4,2.4,0],[-7,-7,0],[-6,-7,0],[-5,-7,0],[-4,-7,0],[3,6,0],[4,6,0],[5,6,0]]
#uav_bias = [[2,-6,0],[-2,-6,0],[2,2,0],[-2,2,0],[4,-6,0],[4,-8,0],[2,-8,0],[3,-7,0],[-4,-6,0],[-4,-8,0],[-2,-8,0],[-3,-7,0],[4,2,0],[4,4,0],[2,4,0],[3,3,0],[-4,2,0],[-4,4,0],[-2,4,0],[-3,3,0]]

class Px4Controller:
    def __init__(self):
        self.uav_id = uav_id + uav_num_real
        self.namespace = "UAV{proc}".format(proc=self.uav_id)

        self.imu = None
        self.gps = None
        self.local_pose = PoseStamped()
        self.local_velocity = TwistStamped()
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 0.6
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None
        self.target_yaw = 0.0
        self.global_pose = PoseStamped()

        self.takeoff_target_pose = PoseStamped()
        self.hover_target_pose = PoseStamped()
        self.target_pose = None
        self.target_vel = TwistStamped()
        self.motion_type = 0  # 0:position 1:vel 2: accel
        self.frame_id = 7  # 1 :flu, 7: position

        self.received_new_task = False
        self.arm_state = False
        self.mavros_state = None
        self.received_imu = False
        self.frame = "BODY"

        self.cmd = None
        self.gcs_cmd = String()
        self.last_gcs_cmd = String()
        self.form_flag = 0
        self.first_form3_flag = True
        self.first_form1_flag = True
        self.first_form2_flag = True
        self.leader_desire_pose = None
        self.last_form = 0

        self.uav_data = UAVDataMsg()
        self.all_uav_data_sim = AllUAVData()
        self.all_uav_data_real = AllUAVData()
        self.all_comm_data = CommVerify()
        # Expand to uav_num lines,add neighbor information in "def read_set_file"
        self.neighbor_num = 0
        self.neighbor_id = []
        self.comm_succ = []
        for i in range(0, uav_num):
            self.neighbor_id.append([])
            self.comm_succ.append([])
        self.leader_id = [0]
        self.all_desired_position = []
        self.all_neighbor_data = AllNeighborMsg()

        # control parameters
        self.Kpx = 0.5
        self.Kpy = 0.5
        self.Kpz = 0.5
        self.Kpangz = 1
        self.velxy_max = 1.5
        self.velz_max = 1.5
        self.velangz_max = 0.5
        self.gamma = 1
        self.integral_x = 0  # integralgrete
        self.integral_y = 0  # integralgrete
        self.integral_z = 0  # integralgrete
        # ros subscribers
        self.local_pose_sub = rospy.Subscriber(self.namespace + "/mavros/local_position/pose", PoseStamped,
                                               self.local_pose_callback)
        self.local_velocity_sub = rospy.Subscriber(self.namespace + "/mavros/local_position/velocity_local",
                                                   TwistStamped, self.local_velocity_callback)
        self.mavros_sub = rospy.Subscriber(self.namespace + "/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber(self.namespace + "/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber(self.namespace + "/mavros/imu/data", Imu, self.imu_callback)
        self.gcs_cmd_sub = rospy.Subscriber("/xtdrone_gcs/cmd", String, self.gcs_cmd_callback)
        self.all_uav_data_sub = rospy.Subscriber("/xtdrone_gcs/all_uav_data", AllUAVData,
                                                     self.all_uav_data_callback)
        self.communication_verify_sub = rospy.Subscriber("/communication_verify", CommVerify, self.communication_verify_callback)

        # ros publishers
        # self.pose_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.local_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_raw/local', PositionTarget,
                                                queue_size=10)
        # self.twist_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.uav_data_pub = rospy.Publisher(self.namespace + '/formation/uav_data', UAVDataMsg, queue_size=10)
        self.global_position_pub = rospy.Publisher(self.namespace + '/global_position/uav_data', PoseStamped,
                                                   queue_size=10)

        # ros services
        self.armService = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
        self.takeoffService = rospy.ServiceProxy(self.namespace + '/mavros/cmd/takeoff', CommandTOL)
        self.landService = rospy.ServiceProxy(self.namespace + '/mavros/cmd/land', CommandTOL)
        self.flightModeService = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        self.frameService = rospy.ServiceProxy(self.namespace + '/mavros/setpoint_velocity/mav_frame', SetMavFrame)
        print(self.namespace, ": Px4 Controller Start!")

        self.data_velocity = []
        self.data_position = []
        self.read_set_file('UAV_pos1')

    def working(self):
        rospy.init_node(self.namespace + "_control_node")
        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print(self.namespace, ": Waiting for initialization.")
                time.sleep(0.5)
        rate = rospy.Rate(15)
        loop_count = 0
        loop_factor = 4
        # self.read_set_file('UAV_pos')
        while rospy.is_shutdown() is False:
            self.neighbor_num = len(self.neighbor_id[self.uav_id])
            loop_count += 1
            self.global_position_pub.publish(self.global_pose)
            self.construct_uav_data()
            self.uav_data_pub.publish(self.uav_data)  # 15Hz
            # if loop_count % 10 == 0:
            #     print(str(self.uav_id)+"'s neighbor number: ", self.neighbor_num)
            if self.gcs_cmd == "AUTO.TAKEOFF":

                self.last_gcs_cmd = "AUTO.TAKEOFF"
                self.form_flag = 0
                self.motion_type = 0
                self.frame_id = 7
                if self.arm_state == False:
                    self.arm()
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                self.target_pose = self.construct_target(x=self.takeoff_target_pose.pose.position.x,
                                                         y=self.takeoff_target_pose.pose.position.y,
                                                         z=self.takeoff_height,
                                                         yaw=self.current_heading)
                self.local_target_pub.publish(self.target_pose)

            elif self.gcs_cmd == 'FORM_1':
                self.motion_type = 1
                self.frame_id = 1
                self.last_gcs_cmd = 'FORM_1'
                if self.form_flag != 1:
                    self.form_flag = 1
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                if self.uav_id == 0 or self.neighbor_num == 0:
                    self.motion_type = 1
                    if self.first_form1_flag:
                        self.leader_desire_pose = copy.deepcopy(self.uav_data)
                        self.last_form = 1
                        self.first_form1_flag = False
                    if self.uav_id == 0:
                        self.leader_desire_pose.pose.position.y += 0.02
                    self.leader_formation_control()
                    self.target_pose = self.construct_target(vx=self.target_vel.twist.linear.x,
                                                             vy=self.target_vel.twist.linear.y,
                                                             vz=self.target_vel.twist.linear.z,
                                                             yaw_rate=self.target_vel.twist.angular.z)
                else:
                    self.leader_desire_pose = copy.deepcopy(self.uav_data)
                    self.formation_control()
                    self.target_pose = self.construct_target(vx=self.target_vel.twist.linear.x,
                                                             vy=self.target_vel.twist.linear.y,
                                                             vz=self.target_vel.twist.linear.z,
                                                             yaw_rate=self.target_vel.twist.angular.z)
                self.local_target_pub.publish(self.target_pose)

            elif self.gcs_cmd == 'FORM_2':
                self.motion_type = 1
                self.frame_id = 1
                self.last_gcs_cmd = 'FORM_2'
                if self.form_flag != 2:
                    self.form_flag = 2
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                if self.uav_id == 0 or self.neighbor_num == 0:
                    self.motion_type = 1
                    if self.first_form2_flag:
                        self.leader_desire_pose = copy.deepcopy(self.uav_data)
                        self.last_form = 2
                        self.first_form2_flag = False
                    if self.uav_id == 0:
                        self.leader_desire_pose.pose.position.y -= 0.02
                    self.leader_formation_control()
                    self.target_pose = self.construct_target(vx=self.target_vel.twist.linear.x,
                                                             vy=self.target_vel.twist.linear.y,
                                                             vz=self.target_vel.twist.linear.z,
                                                             yaw_rate=self.target_vel.twist.angular.z)
                else:
                    self.formation_control()
                    self.leader_desire_pose = copy.deepcopy(self.uav_data)
                    self.target_pose = self.construct_target(vx=self.target_vel.twist.linear.x,
                                                             vy=self.target_vel.twist.linear.y,
                                                             vz=self.target_vel.twist.linear.z,
                                                             yaw_rate=self.target_vel.twist.angular.z)
                self.local_target_pub.publish(self.target_pose)

            elif self.gcs_cmd == 'AUTO.LAND':
                self.last_gcs_cmd = 'AUTO.LAND'
                self.form_flag = 0
                self.last_form = 0
                if self.mavros_state != "AUTO.LAND":
                    self.flight_mode_set(mode='AUTO.LAND')
                if (self.mavros_state == 'AUTO.LAND') and (self.local_pose.pose.position.z < 0.15):
                    if self.arm_state == True:
                        self.disarm()
                    # print(self.namespace, ": Land Success!")
            elif self.gcs_cmd == "HOVER":
                self.last_gcs_cmd = "HOVER"
                self.form_flag = 0
                self.motion_type = 0
                self.frame_id = 7
                if self.arm_state == False:
                    self.arm()
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                self.target_pose = self.construct_target(x=self.hover_target_pose.pose.position.x,
                                                         y=self.hover_target_pose.pose.position.y,
                                                         z=self.hover_target_pose.pose.position.z,
                                                         yaw=self.current_heading)
                self.local_target_pub.publish(self.target_pose)
            else:
                self.gcs_cmd = self.last_gcs_cmd
                self.form_flag = 0
                # self.state = self.flight_mode_set(mode='OFFBOARD')
            # if (self.uav_id == 11) or self.uav_id == 18:
            #     print(str(self.uav_id),"'s neighbors: ", self.neighbor_id[self.uav_id])
            rate.sleep()
        print("over over over")

    def formation_control(self):

        local_desire_relative_pose = self.all_desired_position[self.uav_id]
        delta_x = 0
        delta_vx = 0
        delta_y = 0
        delta_vy = 0
        delta_z = 0
        delta_vz = 0
        self.target_vel.twist.angular.x = 0
        self.target_vel.twist.angular.y = 0
        self.target_vel.twist.angular.z = self.Kpangz * (self.target_yaw - self.current_heading)
        self.neighbor_num = len(self.neighbor_id[self.uav_id])
        if self.neighbor_num > 0:
            weight = 4.0 / (self.gamma * self.gamma*self.neighbor_num)
        else:
            weight = 0.0

        for i in range(self.neighbor_num):
            try:
                delta_x += self.all_uav_data_sim.data[self.neighbor_id[self.uav_id][i]].pose.position.x - self.uav_data.pose.position.x - \
                           self.all_desired_position[self.neighbor_id[self.uav_id][i]][0] + local_desire_relative_pose[0]
                delta_vx += self.all_uav_data_sim.data[self.neighbor_id[self.uav_id][i]].velocity.linear.x - self.uav_data.velocity.linear.x
                delta_y += self.all_uav_data_sim.data[self.neighbor_id[self.uav_id][i]].pose.position.y - self.uav_data.pose.position.y - \
                       self.all_desired_position[self.neighbor_id[self.uav_id][i]][1] + local_desire_relative_pose[1]
                delta_vy += self.all_uav_data_sim.data[self.neighbor_id[self.uav_id][i]].velocity.linear.y - self.uav_data.velocity.linear.y
                delta_z += self.all_uav_data_sim.data[self.neighbor_id[self.uav_id][i]].pose.position.z - self.uav_data.pose.position.z - \
                       self.all_desired_position[self.neighbor_id[self.uav_id][i]][2] + local_desire_relative_pose[2]
                delta_vz += self.all_uav_data_sim.data[self.neighbor_id[self.uav_id][i]].velocity.linear.z - self.uav_data.velocity.linear.z
            except:
                print("id: ",self.uav_id)
                print("neighbor:  ", self.neighbor_id[self.uav_id])
                print("num:  ", self.neighbor_num)

        if self.uav_id == 10:
            print(delta_z)
            print(delta_vz)

        # accel
        self.integral_x = weight * (delta_x + self.gamma * delta_vx)
        self.integral_y = weight * (delta_y + self.gamma * delta_vy)
        self.integral_z = weight * (delta_z + self.gamma * delta_vz)

        # v(k+1)=vk+ak*T
        self.target_vel.twist.linear.x = self.uav_data.velocity.linear.x + self.integral_x * 0.5 * self.Kpx
        self.target_vel.twist.linear.y = self.uav_data.velocity.linear.y + self.integral_y * 0.5 * self.Kpy
        self.target_vel.twist.linear.z = self.uav_data.velocity.linear.z + self.integral_z * 0.5 * self.Kpz

        self.target_vel.twist.linear.x = self.limit_amplitude(self.target_vel.twist.linear.x,
                                                              self.velxy_max)
        self.target_vel.twist.linear.y = self.limit_amplitude(self.target_vel.twist.linear.y,
                                                              self.velxy_max)
        self.target_vel.twist.linear.z = self.limit_amplitude(self.target_vel.twist.linear.z,
                                                              self.velz_max)
        self.target_vel.twist.angular.z = self.limit_amplitude(self.target_vel.twist.angular.z * self.Kpangz,
                                                               self.velangz_max)
        self.data_position.append(
            [self.uav_data.pose.position.x, self.uav_data.pose.position.y, self.uav_data.pose.position.z])
        self.data_velocity.append(
            [self.uav_data.velocity.linear.x, self.uav_data.velocity.linear.y, self.uav_data.velocity.linear.z])
        np.save('data/'+str(self.uav_id) + ' data_position_demo1.txt', self.data_position)
        np.save('data/'+str(self.uav_id) + ' data_velocity_demo1.txt', self.data_velocity)

    def leader_formation_control(self):
        self.target_vel.twist.angular.x = 0
        self.target_vel.twist.angular.y = 0
        self.target_vel.twist.angular.z = self.Kpangz * (self.target_yaw - self.current_heading)
        self.target_vel.twist.linear.x = self.leader_desire_pose.pose.position.x - self.uav_data.pose.position.x
        self.target_vel.twist.linear.y = self.leader_desire_pose.pose.position.y - self.uav_data.pose.position.y
        self.target_vel.twist.linear.z = self.takeoff_height - self.uav_data.pose.position.z
        # print(self.target_vel.twist.linear)
        self.target_vel.twist.linear.x = self.limit_amplitude(self.target_vel.twist.linear.x * 0.7, self.velxy_max)
        self.target_vel.twist.linear.y = self.limit_amplitude(self.target_vel.twist.linear.y * 0.7, self.velxy_max)
        self.target_vel.twist.linear.z = self.limit_amplitude(self.target_vel.twist.linear.z * 0.7, self.velz_max)
        self.target_vel.twist.angular.z = self.limit_amplitude(self.target_vel.twist.angular.z * self.Kpangz,
                                                               self.velangz_max)
        self.data_position.append(
            [self.uav_data.pose.position.x, self.uav_data.pose.position.y, self.uav_data.pose.position.z])
        self.data_velocity.append(
            [self.uav_data.velocity.linear.x, self.uav_data.velocity.linear.y, self.uav_data.velocity.linear.z])
        np.save('data/' + str(self.uav_id) + ' data_position_demo1.txt', self.data_position)
        np.save('data/' + str(self.uav_id) + ' data_velocity_demo1.txt', self.data_velocity)

    def trans_flu2enu(self):
        self.q = np.array(
            [self.local_pose.pose.orientation.x, self.local_pose.pose.orientation.y, self.local_pose.pose.orientation.z,
             self.local_pose.pose.orientation.w])
        Pw = np.array(
            [self.target_vel.twist.linear.x, self.target_vel.twist.linear.y, self.target_vel.twist.linear.z, 0])
        self.q_ = self.qconj(self.q)
        Ps = self.qAd(self.q_, Pw)
        self.target_vel.twist.linear.x = Ps[0]
        self.target_vel.twist.linear.y = Ps[1]
        self.target_vel.twist.linear.z = Ps[2]

    def limit_amplitude(self, data, max_amplitude):
        if max_amplitude < 0:
            print('Warning! Max Amplitude should be positive!')
        if data <= -max_amplitude:
            data = -max_amplitude
        elif data >= max_amplitude:
            data = max_amplitude
        return data

    def communication_verify_callback(self, msg):
        self.all_comm_data = msg
        self.leader_id = msg.leader
        self.neighbor_id[self.uav_id] = []
        self.comm_succ[self.uav_id] = msg.transMatrix[self.uav_id].transMatrix
        for j in range(len(self.leader_id)):
            if self.uav_id in self.leader_id:
                if self.leader_id[j]< self.uav_id and self.comm_succ[self.uav_id][j] == 1:
                    self.neighbor_id[self.uav_id].append(self.leader_id[j])
            else:
                if self.comm_succ[self.uav_id][j] == 1:
                    self.neighbor_id[self.uav_id].append(self.leader_id[j])
        self.neighbor_num = len(self.neighbor_id[self.uav_id])
        # print("UAV",self.uav_id,"'s neighbor_num:   ", self.neighbor_num)


    def read_set_file(self, txt_all_pos):
        all_pos_path = 'txt/' + txt_all_pos + '.txt'
        self.all_desired_position = np.loadtxt(all_pos_path)

    def construct_uav_data(self):
        self.uav_data.header.stamp = rospy.Time.now()
        self.uav_data.uav_id = self.uav_id
        self.uav_data.pose = self.global_pose.pose
        self.uav_data.velocity = self.local_velocity.twist
        self.uav_data.heading = self.current_heading
        self.uav_data.is_sim = 1

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.frame_id

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz

        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        if (self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                        + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                        + PositionTarget.IGNORE_YAW_RATE
        if (self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                        + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                        + PositionTarget.IGNORE_YAW
        if (self.motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                        + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                        + PositionTarget.IGNORE_YAW

        return target_raw_pose

    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''

    def position_distance(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)
        if delta_x + delta_y + delta_z < threshold:
            return True
        else:
            return False

    def cmd_yaw(self, yaw):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.target_pose.pose.orientation.x = quaternion[0]
        self.target_pose.pose.orientation.y = quaternion[1]
        self.target_pose.pose.orientation.z = quaternion[2]
        self.target_pose.pose.orientation.w = quaternion[3]

    def gcs_cmd_callback(self, msg):
        self.gcs_cmd = msg.data

    def all_uav_data_callback(self, msg):

        self.all_uav_data_sim = msg

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
        self.global_pose = copy.deepcopy(msg)
        if self.gcs_cmd != "AUTO.TAKEOFF":
            self.takeoff_target_pose = msg
        if self.gcs_cmd != "HOVER":
            self.hover_target_pose = msg
        if self.gcs_cmd != "FORM_1":
            self.first_form1_flag = True
        if self.gcs_cmd != "FORM_2":
            self.first_form2_flag = True
        if self.gcs_cmd != "FORM_3":
            self.first_form3_flag = True
        if self.uav_id < len(uav_bias):
            self.global_pose.pose.position.x += uav_bias[self.uav_id][0]
            self.global_pose.pose.position.y += uav_bias[self.uav_id][1]
            self.global_pose.pose.position.z += uav_bias[self.uav_id][2]

    def local_velocity_callback(self, msg):
        self.local_velocity = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        self.arm_state = msg.armed

    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    def cmd_vel_flu_callback(self, msg):
        # self.target_vel.twist.linear.x, self.target_vel.twist.linear.y = self.flu2enu(msg.linear.x, msg.linear.y)
        self.target_vel.twist.linear.x = msg.linear.x
        self.target_vel.twist.linear.y = msg.linear.y
        self.target_vel.twist.linear.z = msg.linear.z
        self.target_vel.twist.angular.x = 0
        self.target_vel.twist.angular.y = 0
        self.target_vel.twist.angular.z = msg.angular.z

    def cmd_vel_enu_callback(self, msg):
        self.target_vel.twist = msg

    def gps_callback(self, msg):
        self.gps = msg

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

    def takeoff(self):
        if self.takeoffService(True):
            return True
        else:
            print("Vehicle takeoff failed!")
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

    def qprod(self, q1, q2):
        return tfc.transformations.quaternion_multiply(q1, q2)

    def qconj(self, q):
        return np.hstack((-q[:3], q[3]))

    def qAd(self, q, p):
        return self.qprod(self.qprod(q, p), self.q)


if __name__ == '__main__':
    controller = Px4Controller()
    controller.working()
