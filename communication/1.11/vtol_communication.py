import rospy
import tf
import yaml
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import CommandBool, CommandVtolTransition, SetMode
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import time
from pyquaternion import Quaternion
import math
from multiprocessing import Process
import sys

class Communication:

    def __init__(self, vehicle_type, vehicle_id):
        
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.imu = None
        self.local_pose = None
        self.current_state = None
        self.target_motion = PositionTarget()
        self.global_target = None
        self.arm_state = False
        self.offboard_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
            
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        #self.imu_sub = rospy.Subscriber(self.vehicle_type+self.vehicle_id+"/mavros/imu/data", Imu, self.imu_callback)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback)      
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+self.vehicle_id+"/cmd",String,self.cmd_callback)

        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+self.vehicle_id+"/mavros/set_mode", SetMode)
        self.gazeboModelstate = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        print("communication initialized")

    def start(self):
        rospy.init_node(self.vehicle_type+self.vehicle_id+"_communication")
        rate = rospy.Rate(100)
        '''
        main ROS thread
        '''
        while(rospy.is_shutdown):
            self.target_motion_pub.publish(self.target_motion)
            try:
                response = self.gazeboModelstate (self.vehicle_type+'_'+self.vehicle_id,'ground_plane')
            except rospy.ServiceException, e:
                print "Gazebo model state service call failed: %s"%e
            if (self.flight_mode is "LAND") and (self.local_pose.pose.position.z < 0.15):
                if(self.disarm()):
                    self.flight_mode = "DISARMED"

            rate.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg


    def construct_target(self, x=0, y=0, z=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = 1 

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        if self.mission == 'takeoff':
            target_raw_pose.type_mask = 4096
        elif self.mission == 'land':
            target_raw_pose.type_mask = 8192
        elif self.mission == 'loiter':
            target_raw_pose.type_mask = 12288
        else:
            target_raw_pose.type_mask = 16384

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)
 
    def cmd_pose_enu_callback(self, msg):
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)

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

        elif msg.data == 'takeoff' or msg.data == 'land' or msg.data == 'loiter' or msg.data == 'idle':
            self.mission = msg.data   
            print(self.mission)
        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

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
    def flight_mode_switch(self):
        if self.flightModeService(custom_mode=self.flight_mode):
            print(self.vehicle_type+self.vehicle_id+": "+self.flight_mode)
            return True
        else:
            print(self.vehicle_type+self.vehicle_id+": "+self.flight_mode+"failed")
            return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1],sys.argv[2])
    communication.start()

