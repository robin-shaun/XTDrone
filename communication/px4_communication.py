import rospy
import tf
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import time
from pyquaternion import Quaternion
import math

class PX4Communication:

    def __init__(self):

        self.imu = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 1

        self.target_pose = PoseStamped()
        self.target_vel = TwistStamped()
        self.global_target = None

        self.arm_state = False
        self.offboard_state = False
        self.flag = 0
        self.flight_mode = None

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/cmd_pose_flu", Pose, self.cmd_pose_flu_callback)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/cmd_pose_enu", Pose, self.cmd_pose_enu_callback)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/cmd_vel_flu", Twist, self.cmd_vel_flu_callback)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/cmd_vel_enu", Twist, self.cmd_vel_enu_callback)
        self.cmd_sub = rospy.Subscriber("/xtdrone/cmd", String, self.cmd_callback)

        '''
        ros publishers
        '''
        self.pose_target_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.twist_target_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.odom_groundtruth_pub = rospy.Publisher('/xtdrone/ground_truth/odom', Odometry, queue_size=10)
        self.imu_groundtruth_pub = rospy.Publisher('/xtdrone/ground_truth/imu', Imu, queue_size=10)
        self.pose_groundtruth_pub = rospy.Publisher('/xtdrone/ground_truth/pose', PoseStamped, queue_size=10)
        
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)


        print("PX4 Communication Initialized!")


    def start(self):
        rospy.init_node("px4_communication")
        rate = rospy.Rate(100)
        '''
        main ROS thread
        '''
        while(rospy.is_shutdown):
            if(self.flag==0):
                self.pose_target_pub.publish(self.target_pose)
            else:
                self.twist_target_pub.publish(self.target_vel)
            if (self.flight_mode is "LAND") and (self.local_pose.pose.position.z < 0.15):

                if(self.disarm()):

                    self.flight_mode = "DISARMED"
            try:
                response = self.gazeboModelstate ('iris','ground_plane')
            except rospy.ServiceException, e:
                print "Gazebo model state service call failed: %s"%e
            odom = Odometry()
            odom.header = response.header
            odom.pose.pose = response.pose
            odom.twist.twist = response.twist
            self.odom_groundtruth_pub.publish(odom)
            '''
            imu = Imu()
            imu.header = response.header
            imu.orientation = response.pose.orientation
            imu.angular_velocity = response.twist.angular
            imu.linear_acceleration = 
            '''
            rate.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        self.current_heading = self.q2yaw(msg.orientation)

    def cmd_pose_flu_callback(self, msg):
        self.flag = 0
        self.target_pose.pose.position.x, self.target_pose.pose.position.y = self.flu2enu(msg.position.x, msg.position.y)
        self.target_pose.pose.position.z = msg.position.z
        target_yaw = self.q2yaw(msg.orientation)+self.current_heading
        self.cmd_yaw(target_yaw)
 
    def cmd_pose_enu_callback(self, msg):
        self.flag = 0
        self.target_pose.pose = msg
        target_yaw = self.q2yaw(msg.orientation)
        self.cmd_yaw(target_yaw)

    def cmd_vel_flu_callback(self, msg):
        self.flag = 1
        self.target_vel.twist.linear.x, self.target_vel.twist.linear.y = self.flu2enu(msg.linear.x, msg.linear.y)
        self.target_vel.twist.linear.z = msg.linear.z
        self.target_vel.twist.angular.x = 0
        self.target_vel.twist.angular.y = 0
        self.target_vel.twist.angular.z = msg.angular.z
 
    def cmd_vel_enu_callback(self, msg):
        self.flag = 1
        self.target_vel.twist = msg

    def cmd_callback(self, msg):
        if msg.data == '':
            return
        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print('Armed'+str(self.arm_state))
        elif msg.data == 'DISARM':
            disarm_state =self.disarm()
            if disarm_state:
                self.arm_state = False
            print('Armed'+str(self.arm_state))
        else:
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
            print("Vehicle Arming Failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle Disarming Failed!")
            return False

    def flight_mode_switch(self):
        if self.flightModeService(custom_mode=self.flight_mode):
            print(self.flight_mode)
            return True
        else:
            print(self.flight_mode+"Failed")
            return False

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.3 and self.offboard_state and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':

    px4_com = PX4Communication()
    px4_com.start()
