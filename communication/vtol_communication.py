import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandVtolTransition, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
from pyquaternion import Quaternion
import sys

class Communication:

    def __init__(self, vehicle_type, vehicle_id):
        
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.local_pose = None
        self.current_yaw = 0
        self.current_state = None
        self.target_motion = PositionTarget()
        self.arm_state = False
        self.hover_flag = 0
        self.coordinate_frame = 1
        self.motion_type = 0
        self.flight_mode = None
        self.plane_mission = None
            
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback)
        self.cmd_accel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_flu", Twist, self.cmd_accel_flu_callback)
        self.cmd_accel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_enu", Twist, self.cmd_accel_enu_callback)    
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",String,self.cmd_callback)

        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.odom_groundtruth_pub = rospy.Publisher('/xtdrone/'+self.vehicle_type+'_'+self.vehicle_id+'/ground_truth/odom', Odometry, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode", SetMode)
        self.gazeboModelstate = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.transition = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+'/mavros/cmd/vtol_transition', CommandVtolTransition)
        self.transition_state = 'multirotor'
        print(self.transition_state)
        print(self.transition(state=3))

        print("communication initialized")

    def start(self):
        rospy.init_node(self.vehicle_type+'_'+self.vehicle_id+"_communication")
        rate = rospy.Rate(100)
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            self.target_motion_pub.publish(self.target_motion)
            try:
                response = self.gazeboModelstate (self.vehicle_type+'_'+self.vehicle_id,'ground_plane')
            except rospy.ServiceException, e:
                print "Gazebo model state service call failed: %s"%e
            odom = Odometry()
            odom.header = response.header
            odom.pose.pose = response.pose
            odom.twist.twist = response.twist
            self.odom_groundtruth_pub.publish(odom)
            if (self.flight_mode is "LAND") and (self.local_pose.position.z < 0.15):
                if(self.disarm()):
                    self.flight_mode = "DISARMED"

            rate.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg.pose
        self.current_yaw = self.q2yaw(msg.pose.orientation)

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame

        if self.coordinate_frame == 1:
            target_raw_pose.position.x = x
            target_raw_pose.position.y = y
            target_raw_pose.position.z = z
        else:
            target_raw_pose.position.x = -y
            target_raw_pose.position.y = x
            target_raw_pose.position.z = z
        if self.transition_state == 'plane':
            if self.plane_mission == 'takeoff':
                target_raw_pose.type_mask = 4096
            elif self.plane_mission == 'land':
                target_raw_pose.type_mask = 8192
            elif self.plane_mission == 'loiter':
                target_raw_pose.type_mask = 12288
            else:
                target_raw_pose.type_mask = 16384
        else:
            if self.coordinate_frame == 1: 
                target_raw_pose.velocity.x = vx
                target_raw_pose.velocity.y = vy
                target_raw_pose.velocity.z = vz
                
                target_raw_pose.acceleration_or_force.x = afx
                target_raw_pose.acceleration_or_force.y = afy
                target_raw_pose.acceleration_or_force.z = afz
            else:
                target_raw_pose.velocity.x = -vy
                target_raw_pose.velocity.y = vx
                target_raw_pose.velocity.z = vz
                
                target_raw_pose.acceleration_or_force.x = -afy
                target_raw_pose.acceleration_or_force.y = afx
                target_raw_pose.acceleration_or_force.z = afz

            target_raw_pose.yaw = yaw
            target_raw_pose.yaw_rate = yaw_rate

            if(self.motion_type == 0):
                target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.IGNORE_YAW_RATE
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
        self.coordinate_frame = 9
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)
 
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)
        
    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1     
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)       
 
    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)
            
    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1 
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)
            
    def hover_state_transition(self,x,y,z,w):
        if abs(x) > 0.005 or abs(y) > 0.005 or abs(z) > 0.005 or abs(w) > 0.005:
            self.hover_flag = 0
            
    def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))
            
        elif  msg.data == 'multirotor':
            self.transition_state = msg.data
            print(self.vehicle_type+'_'+self.vehicle_id+':'+msg.data)
            print(self.transition(state=3))
            
        elif  msg.data == 'plane':
            self.transition_state = msg.data
            print(self.vehicle_type+'_'+self.vehicle_id+':'+msg.data)
            print(self.transition(state=4))

        elif msg.data in ['loiter', 'idle']:
            self.plane_mission = msg.data   
            print(self.vehicle_type+'_'+self.vehicle_id+':'+self.plane_mission)
            
        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

    def arm(self):
        if self.armService(True):
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": disarming failed!")
            return False
        
    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.local_pose.position.x,y=self.local_pose.position.y,z=self.local_pose.position.z,yaw=self.current_heading,yaw=self.current_yaw)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
            print(self.vehicle_type+'_'+self.vehicle_id+": Hover")
        elif self.flightModeService(custom_mode=self.flight_mode):
            self.hover_flag = 0
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode)
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+"Failed")
            return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1],sys.argv[2])
    communication.start()

