import rospy
import tf
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist, Vector3Stamped, Vector3
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import time
from pyquaternion import Quaternion
import math
from multiprocessing import Process
import sys

class PX4Communication:

    def __init__(self,id):

        self.id = id
        self.imu = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 1
        self.hover_flag = 0

        self.target_motion = PositionTarget()
        self.global_target = None

        self.arm_state = False
        self.offboard_state = False
        self.flag = 0
        self.flight_mode = None
        self.mission = None

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/uav"+str(self.id)+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/uav"+str(self.id)+"/mavros/state", State, self.mavros_state_callback)
        self.imu_sub = rospy.Subscriber("/uav"+str(self.id)+"/mavros/imu/data", Imu, self.imu_callback)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+"uav"+str(self.id)+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+"uav"+str(self.id)+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+"uav"+str(self.id)+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+"uav"+str(self.id)+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback)
        self.cmd_accel_flu_sub = rospy.Subscriber("/xtdrone/"+"uav"+str(self.id)+"/cmd_accel_flu", Vector3, self.cmd_accel_flu_callback)
        self.cmd_accel_enu_sub = rospy.Subscriber("/xtdrone/"+"uav"+str(self.id)+"/cmd_accel_enu", Vector3, self.cmd_accel_enu_callback)
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+"uav"+str(self.id)+"/cmd",String,self.cmd_callback)

        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher("/uav"+str(self.id)+"/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy("/uav"+str(self.id)+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy("/uav"+str(self.id)+"/mavros/set_mode", SetMode)


        print("UAV"+str(self.id)+": "+"PX4 Communication Initialized!")


    def start(self):
        rospy.init_node("px4_uav"+str(self.id)+"_communication")
        rate = rospy.Rate(100)
        '''
        main ROS thread
        '''
        while(rospy.is_shutdown):
            self.target_motion_pub.publish(self.target_motion)
            if (self.flight_mode is "LAND") and (self.local_pose.pose.position.z < 0.15):

                if(self.disarm()):

                    self.flight_mode = "DISARMED"

            rate.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        self.current_heading = self.q2yaw(msg.orientation)

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = 1

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

        if(self.flag == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        if(self.flag == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        if(self.flag == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.flag = 0
        target_x_enu, target_y_enu = self.flu2enu(msg.position.x, msg.position.y)
        target_z_enu = msg.position.z
        #target_yaw = self.q2yaw(msg.orientation)+self.current_heading
        self.target_motion = self.construct_target(x=target_x_enu,y=target_y_enu,z=target_z_enu)
 
    def cmd_pose_enu_callback(self, msg):
        self.flag = 0
        #target_yaw = self.q2yaw(msg.orientation)+self.current_heading
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)

    def cmd_vel_flu_callback(self, msg):
        if self.hover_flag == 0:
            self.flag = 1
            target_vx_enu, target_vy_enu = self.flu2enu(msg.linear.x, msg.linear.y)
            target_vz_enu = msg.linear.z
            target_yaw_rate = msg.angular.z
            self.target_motion = self.construct_target(vx=target_vx_enu,vy=target_vy_enu,vz=target_vz_enu,yaw_rate=target_yaw_rate)
 
    def cmd_vel_enu_callback(self, msg):
        if self.hover_flag == 0:
            self.flag = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)

    def cmd_accel_flu_callback(self, msg):
        if self.hover_flag == 0:
            self.flag = 2
            target_afx_enu, target_afy_enu = self.flu2enu(msg.x, msg.y)
            target_afz_enu = msg.z
            self.target_motion = self.construct_target(afx=target_afx_enu,afy=target_afy_enu,afz=target_afz_enu)
    def cmd_accel_enu_callback(self, msg):
        if self.hover_flag == 0:
            self.flag = 2
            self.target_motion = self.construct_target(afx=msg.x,afy=msg.y,afz=msg.z)

    def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print("UAV"+str(self.id)+": "+'Armed'+str(self.arm_state))

        elif msg.data == 'DISARM':
            disarm_state =self.disarm()
            if disarm_state:
                self.arm_state = False
            print("UAV"+str(self.id)+": "+'Armed'+str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print("UAV"+str(self.id)+": "+msg.data)

        elif not msg.data == self.flight_mode:
            self.flight_mode = msg.data
            self.flight_mode_switch()
            

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def flu2enu(self, x_flu, y_flu):
        x_enu =  x_flu*math.cos(self.current_heading)-y_flu*math.sin(self.current_heading)
        y_enu =  x_flu*math.sin(self.current_heading)+y_flu*math.cos(self.current_heading)
        return x_enu, y_enu

    def cmd_yaw(self, yaw):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.target_pose.pose.orientation.x = quaternion[0]
        self.target_pose.pose.orientation.y = quaternion[1]
        self.target_pose.pose.orientation.z = quaternion[2]
        self.target_pose.pose.orientation.w = quaternion[3]

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("UAV"+str(self.id)+": Vehicle Arming Failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("UAV"+str(self.id)+": Vehicle Disarming Failed!")
            return False

    def hover(self):
        self.flag = 0
        self.target_motion = self.construct_target(x=self.local_pose.pose.position.x,y=self.local_pose.pose.position.y,z=self.local_pose.pose.position.z)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
            print("UAV"+str(self.id)+": Hover")
        elif self.flightModeService(custom_mode=self.flight_mode):
            self.hover_flag = 0
            print("UAV"+str(self.id)+": "+self.flight_mode)
            return True
        else:
            print("UAV"+str(self.id)+": "+self.flight_mode+"Failed")
            return False

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.3 and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':
    uav_num = int(sys.argv[1])
    for i in range(uav_num):
        px4_com = PX4Communication(i+1)
        p = Process(target=px4_com.start)
        p.start()
