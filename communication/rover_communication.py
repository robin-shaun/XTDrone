import rospy
from mavros_msgs.msg import State, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
import sys

rospy.init_node('rover_'+sys.argv[1]+"_communication")
rate = rospy.Rate(30)

class Communication:

    def __init__(self, vehicle_id):
        self.vehicle_type = 'rover'
        self.vehicle_id = vehicle_id
        self.local_pose = None
        self.coordinate_frame = 1
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = self.coordinate_frame
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.motion_type = 1

        # MAVROS Connection Check
        mavros_state = rospy.wait_for_message(self.vehicle_type+'_'+self.vehicle_id+"/mavros/state", State)
        if not mavros_state.connected:
            rospy.logwarn(self.vehicle_type+'_'+self.vehicle_id+": No connection to FCU. Check mavros!")
            exit(0)
            
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=1)
        self.mavros_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/state", State, self.mavros_state_callback,queue_size=1)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback,queue_size=1)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback,queue_size=1)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback,queue_size=1)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback,queue_size=1)           
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",String,self.cmd_callback,queue_size=3)

        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode", SetMode)
        self.set_param_srv = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/param/set", ParamSet)
        rcl_except = ParamValue(4, 0.0)
        self.set_param_srv("COM_RCL_EXCEPT", rcl_except)

        print(self.vehicle_type+'_'+self.vehicle_id+": "+"communication initialized")

    def start(self):
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            self.target_motion_pub.publish(self.target_motion)
            rate.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, yaw=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame 

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz

        target_raw_pose.yaw = yaw

        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        if(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame=9
        self.motion_type=0
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=0)
 
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame=1
        self.motion_type=0
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=0)

    def cmd_vel_flu_callback(self, msg):
        self.coordinate_frame=8
        self.motion_type=1
        self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.angular.z,vz=msg.linear.z,yaw=0)
 
    def cmd_vel_enu_callback(self, msg):
        self.coordinate_frame=1
        self.motion_type=1
        self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.angular.z,vz=msg.linear.z,yaw=0)

    def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state =not self.disarm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print(self.vehicle_type+'_'+self.vehicle_id+": "+msg.data)

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

    def flight_mode_switch(self):
        if self.flightModeService(custom_mode=self.flight_mode):
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode)
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+"failed")
            return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1])
    communication.start()
