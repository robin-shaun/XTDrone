# -*- coding: UTF-8 -*-
"""
Framework for Parallel Simulation

* To control all UAV in Gazebo(simulation environment) using MPI

Before running this code, you need to launch your simulation Environment and px4 via:
$ roslaunch px4 iris3_parallel_simulation.launch    # 3 UAVs in simulation
$ roslaunch px4 iris10_parallel_simulation.launch   # 10 UAVs in simulation

And then, you can run this code via:
$ mpiexec -n 3 python px4_control_sim.py            # 3 UAVs in simulation
$ mpiexec -n 10 python px4_control_sim.py           # 10 UAVs in simulation
"""

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
from control.msg import UAVDataMsg, AllUAVData, NeighborMsg, AllNeighborMsg
import numpy as np
import copy
import tf_conversions as tfc

comm = MPI.COMM_WORLD
uav_id = comm.Get_rank()
uav_num_sim = comm.Get_size()
uav_num_real = 0
uav_num = uav_num_sim + uav_num_real
uav_bias = [[0,0,0],[-1,0,0],[1,0,0],[0,2,0],[0,-2,0],[0,3,0]]


class Px4Controller:
    def __init__(self):
        self.uav_id = uav_id
        self.namespace = "UAV{proc}".format(proc=self.uav_id)

        self.imu = None
        self.gps = None
        self.local_pose = None
        self.local_velocity = None
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 1
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None
        self.target_yaw = 0.0

        self.takeoff_target_pose = PoseStamped()
        self.hover_target_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_vel = TwistStamped()

        self.received_new_task = False
        self.arm_state = False
        self.mavros_state = None
        self.received_imu = False
        self.frame = "BODY"

        self.cmd = None
        self.gcs_cmd = String()
        self.last_gcs_cmd = String()
        self.form_flag = 0
        self.first_hover_flag = False

        self.uav_data = UAVDataMsg()
        self.all_uav_data_sim = AllUAVData()
        self.all_uav_data_real = AllUAVData()
        # Expand to uav_num lines,add neighbor information in "def read_set_file"
        self.neighbor_id =[]
        for i in range(0,uav_num):
            self.neighbor_id.append([])
        self.all_desired_position = []
        self.all_neighbor_data = AllNeighborMsg()

        # control parameters
        self.Kpx = 0.2
        self.Kpy = 0.2
        self.Kpz = 0.2
        self.Kpangz = 1
        self.velxy_max = 1
        self.velz_max = 1
        self.velangz_max = 0.5

        # ros subscribers
        self.local_pose_sub = rospy.Subscriber(self.namespace + "/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_velocity_sub = rospy.Subscriber(self.namespace + "/mavros/local_position/velocity_local", TwistStamped, self.local_velocity_callback)
        self.mavros_sub = rospy.Subscriber(self.namespace + "/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber(self.namespace + "/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber(self.namespace + "/mavros/imu/data", Imu, self.imu_callback)
        self.gcs_cmd_sub = rospy.Subscriber("/xtdrone_gcs/cmd", String, self.gcs_cmd_callback)
        self.all_uav_data_sim_sub = rospy.Subscriber("/xtdrone_gcs/all_uav_data/sim", AllUAVData, self.all_uav_data_sim_callback)
        self.all_uav_data_real_sub = rospy.Subscriber("/xtdrone_gcs/all_uav_data/real", AllUAVData, self.all_uav_data_real_callback)

        # ros publishers
        self.pose_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.local_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.twist_target_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.uav_data_pub = rospy.Publisher(self.namespace + '/formation/uav_data', UAVDataMsg, queue_size=10)

        # ros services
        self.armService = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
        self.takeoffService = rospy.ServiceProxy(self.namespace + '/mavros/cmd/takeoff', CommandTOL)
        self.landService = rospy.ServiceProxy(self.namespace + '/mavros/cmd/land', CommandTOL)
        self.flightModeService = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        self.frameService = rospy.ServiceProxy(self.namespace + '/mavros/setpoint_velocity/mav_frame', SetMavFrame)
        print(self.namespace, ": Px4 Controller Start!")

    def working(self):
        rospy.init_node(self.namespace + "_control_node")
        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print(self.namespace, ": Waiting for initialization.")
                time.sleep(0.5)

        loop_count = 0
        loop_factor = 4
        while rospy.is_shutdown() is False:
            loop_count += 1
            if loop_count % loop_factor == 0:
                if loop_count == loop_factor:
                    loop_count = 0
                self.construct_uav_data()
                self.uav_data_pub.publish(self.uav_data)    # 15Hz

            if self.gcs_cmd == "AUTO.TAKEOFF":
                self.last_gcs_cmd = "AUTO.TAKEOFF"
                self.form_flag = 0
                if self.arm_state == False:
                    self.arm()
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                self.target_pose = self.construct_target(self.takeoff_target_pose.pose.position.x,
                                                         self.takeoff_target_pose.pose.position.y , self.takeoff_height,
                                                         self.current_heading)
                self.local_target_pub.publish(self.target_pose)
                if self.takeoff_detection():
                    print(self.namespace, ": Takeoff Success!")
                else:
                    print(self.namespace, ": Takeoff Failed!")
                self.frameService(8)

            elif self.gcs_cmd == 'FORM_1':
                self.last_gcs_cmd = 'FORM_1'
                if self.form_flag != 1:
                    self.read_set_file('FORM_1_id','FORM_1_pos')
                self.form_flag = 1
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                self.formation_control()
                self.twist_target_pub.publish(self.target_vel)

            elif self.gcs_cmd == 'FORM_2':
                self.last_gcs_cmd = 'FORM_2'
                if self.form_flag != 2:
                    self.read_set_file('FORM_2_id','FORM_2_pos')
                self.form_flag = 2
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                self.formation_control()
                self.twist_target_pub.publish(self.target_vel)

            elif self.gcs_cmd == 'AUTO.LAND':
                self.last_gcs_cmd = 'AUTO.LAND'
                self.form_flag = 0
                if self.mavros_state != "AUTO.LAND":
                    self.flight_mode_set(mode='AUTO.LAND')
                if (self.mavros_state == 'AUTO.LAND') and (self.local_pose.pose.position.z < 0.15):
                    if self.arm_state == True:
                        self.disarm()
                    print(self.namespace, ": Land Success!")
            elif self.gcs_cmd == "HOVER":
                self.last_gcs_cmd = "HOVER"
                self.form_flag = 0
                if self.first_hover_flag:
                    hover_pose = self.construct_target(self.hover_target_pose.pose.position.x,
                                                         self.hover_target_pose.pose.position.y, self.hover_target_pose.pose.position.z,
                                                         self.current_heading)
                    self.first_hover_flag = False
                if self.arm_state == False:
                    self.arm()
                if self.mavros_state != "OFFBOARD":
                    self.flight_mode_set(mode='OFFBOARD')
                self.target_pose = hover_pose
                self.local_target_pub.publish(self.target_pose)
            else:
                self.gcs_cmd = self.last_gcs_cmd
                self.form_flag = 0
                # self.state = self.flight_mode_set(mode='OFFBOARD')

            time.sleep(0.15)    # 60Hz

    def formation_control(self):
        neighbor_num = len(self.all_neighbor_data.data)
        local_desire_relative_pose = self.all_desired_position[self.uav_id]
        self.target_vel.twist.linear.x = 0
        self.target_vel.twist.linear.y = 0
        self.target_vel.twist.linear.z = 0
        self.target_vel.twist.angular.x = 0
        self.target_vel.twist.angular.y = 0
        self.target_vel.twist.angular.z = self.Kpangz*(self.target_yaw-self.current_heading)
        # print("neighbor_num",neighbor_num)
        for i in range(neighbor_num):
            if self.uav_id == 2:
                print(i)
                print(self.all_neighbor_data.data[i].uav_data.pose.position.x- self.uav_data.pose.position.x)
            self.target_vel.twist.linear.x += self.all_neighbor_data.data[i].uav_data.pose.position.x - self.uav_data.pose.position.x - \
                                                self.all_neighbor_data.data[i].dersire_rela_pose.x + local_desire_relative_pose[0]
            self.target_vel.twist.linear.y += self.all_neighbor_data.data[i].uav_data.pose.position.y - self.uav_data.pose.position.y - \
                                                self.all_neighbor_data.data[i].dersire_rela_pose.y + local_desire_relative_pose[1]
            self.target_vel.twist.linear.z += self.all_neighbor_data.data[i].uav_data.pose.position.z - self.uav_data.pose.position.z - \
                                                self.all_neighbor_data.data[i].dersire_rela_pose.z + local_desire_relative_pose[2]
        # print(self.target_vel.twist.linear)
        self.target_vel.twist.linear.x = self.limit_amplitude(self.target_vel.twist.linear.x * self.Kpx, self.velxy_max)
        self.target_vel.twist.linear.y = self.limit_amplitude(self.target_vel.twist.linear.y * self.Kpy, self.velxy_max)
        self.target_vel.twist.linear.z = self.limit_amplitude(self.target_vel.twist.linear.z * self.Kpz, self.velz_max)
        self.target_vel.twist.angular.z = self.limit_amplitude(self.target_vel.twist.angular.z * self.Kpangz, self.velangz_max)
        # self.trans_flu2enu()

    def trans_flu2enu(self):
        q = np.array([self.local_pose.pose.orientation.x, self.local_pose.pose.orientation.y, self.local_pose.pose.orientation.z, self.local_pose.pose.orientation.w])
        Pw = np.array([0, self.target_vel.twist.linear.x, self.target_vel.twist.linear.y, self.target_vel.twist.linear.z])
        Ps = self.qAd(self.qconj(q),Pw)
        self.target_vel.twist.linear.x = Ps[0]
        self.target_vel.twist.linear.x = Ps[1]
        self.target_vel.twist.linear.x = Ps[2]

    def limit_amplitude(self, data, max_amplitude):
        if max_amplitude < 0:
            print('Warning! Max Amplitude should be positive!')
        if data <= -max_amplitude:
            data = -max_amplitude
        elif data >= max_amplitude:
            data = max_amplitude
        return data

    def read_set_file(self,txt_id,txt_all_pos):
        """
        Todo: Feng Yi
        self.neighbor_id =
        self.all_desired_position =
        """
        self.neighbor_id =[]
        for i in range(0,uav_num):
            self.neighbor_id.append([])
        id_path='txt/'+txt_id+'.txt'
        all_pos_path='txt/'+txt_all_pos+'.txt'
        txt_uav_neighbor_num = np.loadtxt(id_path,dtype=int)
        self.all_desired_position = np.loadtxt(all_pos_path)
        for i in range(0, len(txt_uav_neighbor_num[:, 1])):
            for j in range(0, uav_num):
                if txt_uav_neighbor_num[i, 0] == j and txt_uav_neighbor_num[i, 1] != j:
                   self.neighbor_id[txt_uav_neighbor_num[i, 0]].append(txt_uav_neighbor_num[i, 1])
        self.all_neighbor_data = AllNeighborMsg()
        self.all_neighbor_data.data = [NeighborMsg() for i in range(len(self.neighbor_id[self.uav_id]))]

    def construct_uav_data(self):
        self.uav_data.header.stamp = rospy.Time.now()
        self.uav_data.uav_id = self.uav_id
        if self.uav_id<len(uav_bias):
            self.local_pose.pose.position.x += uav_bias[self.uav_id][0]
            self.local_pose.pose.position.y += uav_bias[self.uav_id][1]
            self.local_pose.pose.position.z += uav_bias[self.uav_id][2]
        self.uav_data.pose = self.local_pose.pose
        self.uav_data.velocity = self.local_velocity.twist
        self.uav_data.heading = self.current_heading
        self.uav_data.is_sim = 1

    def construct_target(self, x, y, z, yaw, yaw_rate = 0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 7

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z
        target_raw_pose.yaw = -1.0  # 将yaw改为0.0
        target_raw_pose.yaw_rate = yaw_rate

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE \
                                    + PositionTarget.FORCE
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

    def all_uav_data_sim_callback(self, msg):
        self.all_uav_data_sim = msg
        """
        Todo: Jiarun Yan
        self.neighbor_data = 
        self.all_neighbor_data = 
        """
        neighbor_data = NeighborMsg()
        for i in range(len(self.all_uav_data_sim.data)):
            if self.all_uav_data_sim.data[i].uav_id in self.neighbor_id[self.uav_id]:
                neighbor_data.uav_data = self.all_uav_data_sim.data[i]
                neighbor_data.dersire_rela_pose.x = self.all_desired_position[self.all_uav_data_sim.data[i].uav_id][0]
                neighbor_data.dersire_rela_pose.y = self.all_desired_position[self.all_uav_data_sim.data[i].uav_id][1]
                neighbor_data.dersire_rela_pose.z = self.all_desired_position[self.all_uav_data_sim.data[i].uav_id][2]
                self.all_neighbor_data.data[self.neighbor_id[self.uav_id].index(self.all_uav_data_sim.data[i].uav_id)] = copy.deepcopy(neighbor_data)

    def all_uav_data_real_callback(self, msg):
        self.all_uav_data_real = msg
        """
        Todo: Jiarun Yan
        self.neighbor_data = 
        self.all_neighbor_data = 
        """
        neighbor_data = NeighborMsg()
        for i in range(len(self.all_uav_data_real.data)):
            if self.all_uav_data_real.data[i].uav_id in self.neighbor_id[self.uav_id]:
                neighbor_data.uav_data = self.all_uav_data_real.data[i]
                neighbor_data.dersire_rela_pose.x = self.all_desired_position[self.all_uav_data_real.data[i].uav_id][0]
                neighbor_data.dersire_rela_pose.y = self.all_desired_position[self.all_uav_data_real.data[i].uav_id][1]
                neighbor_data.dersire_rela_pose.z = self.all_desired_position[self.all_uav_data_real.data[i].uav_id][2]
                self.all_neighbor_data.data[self.neighbor_id[self.uav_id].index(self.all_uav_data_real.data[i].uav_id)] =  copy.deepcopy(neighbor_data)


    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
        if self.gcs_cmd != "AUTO.TAKEOFF":
            self.takeoff_target_pose = msg
        if self.gcs_cmd !="HOVER":
            self.hover_target_pose = msg
            self.first_hover_flag = True


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

    def flu2enu(self, x_flu, y_flu):
        x_enu =  x_flu*math.cos(self.current_heading)-y_flu*math.sin(self.current_heading)
        y_enu =  x_flu*math.sin(self.current_heading)+y_flu*math.cos(self.current_heading)
        return x_enu, y_enu

    def FLU2ENU(self, msg):
        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z

    def set_target_position_callback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg)

            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)
        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)

    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_deg)

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
        return self.qprod(self.qprod(q, p), self.qconj(q))





if __name__ == '__main__':
    controller = Px4Controller()
    controller.working()
