import rospy
import tf
import yaml
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandVtolTransition, SetMode
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist, Vector3Stamped, Vector3
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import time
from pyquaternion import Quaternion
import math
from multiprocessing import Process
import sys

class Communication:

    def __init__(self, vehicle_id):
        
        if vehicle_type == 'iris' or 'typhoon' or 'solo':
            self.vehicle_type = vehicle_type
        else:
            print('only support iris, typhoon and solo for multirotors')
            sys.exit()
        self.vehicle_id = vehicle_id
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
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.transition_state = None
        self.transition = None
            
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber(self.vehicle_type+self.vehicle_id+"/mavros/state", State, self.mavros_state_callback)
        self.imu_sub = rospy.Subscriber(self.vehicle_type+self.vehicle_id+"/mavros/imu/data", Imu, self.imu_callback)
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+self.vehicle_id+"/cmd",String,self.cmd_callback)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+str(self.vehicle_id)+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+str(self.vehicle_id)+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+str(self.vehicle_id)+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+str(self.vehicle_id)+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback)
        self.cmd_accel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+str(self.vehicle_id)+"/cmd_accel_flu", Vector3, self.cmd_accel_flu_callback)
        self.cmd_accel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+str(self.vehicle_id)+"/cmd_accel_enu", Vector3, self.cmd_accel_enu_callback)
        self.odom_groundtruth_pub = rospy.Publisher('/xtdrone/ground_truth/odom', Odometry, queue_size=10)
            
        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.odom_groundtruth_pub = rospy.Publisher('/xtdrone/ground_truth/odom', Odometry, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+self.vehicle_id+"/mavros/set_mode", SetMode)
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        if self.vehicle_type == 'tiltrotor' or 'tailsitter' or 'standard_vtol':
            self.transition = rospy.ServiceProxy('/mavros/cmd/vtol_transition', CommandVtolTransition)
            self.transition_state = 'multirotor'
            print(self.transition(state = 3))

        print(self.vehicle_type+self.vehicle_id+": "+"communication initialized")


    def start(self):
        rospy.init_node(self.vehicle_type+self.vehicle_id+"_communication")
        rate = rospy.Rate(100)
        '''
        main ROS thread
        '''
        while(rospy.is_shutdown):
            self.target_motion_pub.publish(self.target_motion)
            
            if (self.flight_mode is "LAND") and (self.local_pose.pose.position.z < 0.15):
                if(self.disarm()):
                    self.flight_mode = "DISARMED"
                    
            try:
                response = self.gazeboModelstate (self.vehicle+self.vehicle_id,'ground_plane')
            except rospy.ServiceException, e:
                print "Gazebo model state service call failed: %s"%e
            odom = Odometry()
            odom.header = response.header
            odom.pose.pose = response.pose
            odom.twist.twist = response.twist
            self.odom_groundtruth_pub.publish(odom)

            rate.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        self.current_heading = self.q2yaw(msg.orientation)

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        if self.vehicle_type == 'iris' or self.vehicle_type = 'solo' or self.vehicle_type = 'typhoon': 
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

            if(self.motion_type == 0):
                target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.IGNORE_YAW
            if(self.motion_type == 1):
                target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.IGNORE_YAW
            if(self.motion_type == 2):
                target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_YAW

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.motion_type = 0
        target_x_enu, target_y_enu = self.flu2enu(msg.position.x, msg.position.y)
        target_z_enu = msg.position.z
        #target_yaw = self.q2yaw(msg.orientation)+self.current_heading
        self.target_motion = self.construct_target(x=target_x_enu,y=target_y_enu,z=target_z_enu)
 
    def cmd_pose_enu_callback(self, msg):
        self.motion_type = 0
        #target_yaw = self.q2yaw(msg.orientation)+self.current_heading
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)

    def cmd_vel_flu_callback(self, msg):
        if self.hover_flag == 0:
            self.motion_type = 1
            target_vx_enu, target_vy_enu = self.flu2enu(msg.linear.x, msg.linear.y)
            target_vz_enu = msg.linear.z
            target_yaw_rate = msg.angular.z
            self.target_motion = self.construct_target(vx=target_vx_enu,vy=target_vy_enu,vz=target_vz_enu,yaw_rate=target_yaw_rate)
 
    def cmd_vel_enu_callback(self, msg):
        if self.hover_flag == 0:
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)

    def cmd_accel_flu_callback(self, msg):
        if self.hover_flag == 0:
            self.motion_type = 2
            target_afx_enu, target_afy_enu = self.flu2enu(msg.x, msg.y)
            target_afz_enu = msg.z
            self.target_motion = self.construct_target(afx=target_afx_enu,afy=target_afy_enu,afz=target_afz_enu)
    def cmd_accel_enu_callback(self, msg):
        if self.hover_flag == 0:
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.x,afy=msg.y,afz=msg.z)

    def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print(self.vehicle_type+self.vehicle_id+": "+'armed'+str(self.arm_state))

        elif msg.data == 'DISARM':
            disarm_state =self.disarm()
            if disarm_state:
                self.arm_state = False
            print(self.vehicle_type+self.vehicle_id+": "+'armed'+str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print(self.vehicle_type+self.vehicle_id+": "+msg.data)

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
            print(self.vehicle_type+self.vehicle_id+": arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print(self.vehicle_type+self.vehicle_id+": disarming failed!")
            return False

    def hover(self):
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.local_pose.pose.position.x,y=self.local_pose.pose.position.y,z=self.local_pose.pose.position.z)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
            print(self.vehicle_type+self.vehicle_id+":"+self.flight_mode)
        elif self.flightModeService(custom_mode=self.flight_mode):
            self.hover_flag = 0
            print(self.vehicle_type+self.vehicle_id+": "+self.flight_mode)
            return True
        else:
            print(self.vehicle_type+self.vehicle_id+": "+self.flight_mode+"failed")
            return False

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.3 and self.arm_state:
            return True
        else:
            return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1],sys.argv[2])
    communication.start()
