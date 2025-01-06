#-*- coding: UTF-8 -*-
import rospy
import tty, termios
import sys,select
from mavros_msgs.msg import AttitudeTarget,PositionTarget,State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
import time
import PyKDL
import math
import numpy as np
import sys


class pub:
    def __init__(self, uav_type, control_type):
        self.px = 0
        self.py = 0
        self.pz = 0
        self.vx,self.vy,self.vz = 0,0,0
        self.roll,self.pitch,self.yaw=0,0,0
        self.roll_rate,self.pitch_rate,self.yaw_rate=0,0,0
        self.arm_state = False
        self.mavros_state = State()
        self.control_type = control_type
 
        self.msg2leader =  '''
        4/1: increase/decrease px_r
        5/2:increase/decrease py_r
        6/3:increase/decrease pz_r
        7/8:increase/decrease yaw_r
        a:arm
        d:disarm
        r:return  
        l:land
        b:begin attitude_contorl
        p:print key_contorl msg
        q: quit
         '''
        # subscribers
        self.local_pose_sub = rospy.Subscriber(uav_type + "_0/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber(uav_type + "_0/mavros/local_position/velocity_body",TwistStamped, self.local_vel_callback)
        self.odom_sub=rospy.Subscriber(uav_type + "_0/mavros/local_position/odom",Odometry,self.odom_callback)
        self.mavros_sub = rospy.Subscriber(uav_type + "_0/mavros/state", State, self.mavros_state_callback)

        # pub
        self.target_motion_pub = rospy.Publisher(uav_type + "_0/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.body_target_pub = rospy.Publisher(uav_type + "_0/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=2)
        self.flightModeService = rospy.ServiceProxy(uav_type + "_0/mavros/set_mode", SetMode)
        self.armService = rospy.ServiceProxy(uav_type + "_0/mavros/cmd/arming", CommandBool)

        self.set_param_srv = rospy.ServiceProxy(uav_type+"_0/mavros/param/set", ParamSet)
        rcl_except = ParamValue(4, 0.0)
        self.set_param_srv("COM_RCL_EXCEPT", rcl_except)

    def local_pose_callback(self, msg):
        self.px = msg.pose.position.x
        self.py = -msg.pose.position.y
        self.pz = msg.pose.position.z

    def local_vel_callback(self, msg):
        self.vx=msg.twist.linear.x
        self.vy=-msg.twist.linear.y
        self.vz=msg.twist.linear.z

    def odom_callback(self,msg):
        x=msg.pose.pose.orientation.x
        y=msg.pose.pose.orientation.y
        z=msg.pose.pose.orientation.z
        w=msg.pose.pose.orientation.w
        rot = PyKDL.Rotation.Quaternion(x,y,z,w)
        self.roll =rot.GetRPY()[0]
        self.pitch =rot.GetRPY()[1]
        self.yaw =rot.GetRPY()[2]

        self.roll_rate=msg.twist.twist.angular.x
        self.pitch_rate=msg.twist.twist.angular.y
        self.yaw_rate=msg.twist.twist.angular.z
        
    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        self.arm_state = msg.armed

    def arm(self):
        if self.armService(True):
            print('arm')
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            print('disarm')
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def euler_to_quaternion(self,roll, pitch, yaw):
        ox=math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        oy=math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        oz=math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        ow=math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        return np.array([ow, ox, oy, oz])

    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def start(self):
        self.time_start=time.time()
        rospy.init_node("attitude_contorl_node")        
        print('start!')
        print(self.msg2leader)
        offborad_mission=None
        self.att =AttitudeTarget()
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        takeoff_x,takeoff_y,takeoff_z,takeoff_yaw=0,0,2,0
        control_x,control_y,control_z,yaw_r=1,0,2,0
        angle_max=0.3

        while not rospy.is_shutdown():          
            self.time_now=time.time()
            self.clock=self.time_now-self.time_start
            key = self.getKey() 
            if  key == '4' :
                control_x =control_x +0.2
                print(self.msg2leader)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == '1' :
                control_x =control_x -0.2
                print(self.msg2leader)
                print('control_x=',control_x)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == '5' :
                control_y =control_y +0.2
                print(self.msg2leader)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == '2' :
                control_y =control_y -0.2
                print(self.msg2leader)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == '6' :
                control_z =control_z +0.2
                print(self.msg2leader)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == '3' :
                control_z =control_z -0.2
                print(self.msg2leader)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == '7' :
                yaw_r =yaw_r +0.05
                print(self.msg2leader)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == '8' :
                yaw_r =yaw_r -0.05
                print(self.msg2leader)
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == 'a' :                
                self.arm()
            elif key == 'd' :
                self.disarm()
            elif key == 'r' :  
                offborad_mission='return'
                print('return to takeoff_target',target_raw_pose.position)
            elif key == 'h':
                self.flightModeService(custom_mode='HOVER')                
                print('HOVER ')
            elif key == 'b':
                offborad_mission='contorl'               
                print('begin attitude_contorl')
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
            elif key == 't' : 
                self.arm()         
                offborad_mission='takeoff'
                print(self.msg2leader) 
                print('offborad_mission=',offborad_mission)               
                print("takeoff_x:%.2f   takeoff_y: %.2f    takeoff_z: %.2f " % (takeoff_x,takeoff_y,takeoff_z))
            elif key == 'l':
                self.flightModeService(custom_mode='AUTO.LAND') 
                offborad_mission='land'                
                print(self.msg2leader)
                print('AUTO.LAND contorl')    
            elif key == 'p' :
                print(self.msg2leader) 
                print("control_x:%.2f   control_y: %.2f    control_z: %.2f yaw_r: %.2f " % (control_x,control_y,control_z,yaw_r))
                print("takeoff_x:%.2f   takeoff_y: %.2f    takeoff_z: %.2f " % (takeoff_x,takeoff_y,takeoff_z))
            elif key == 'q' :
                break
                 
            if offborad_mission in['return','takeoff']:
                self.flightModeService(custom_mode='OFFBOARD')
                target_raw_pose.position.x = takeoff_x
                target_raw_pose.position.y = takeoff_y
                target_raw_pose.position.z = takeoff_z
                target_raw_pose.yaw = takeoff_yaw
                target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE
                self.target_motion_pub.publish(target_raw_pose)
            if offborad_mission=='contorl': 
                self.flightModeService(custom_mode='OFFBOARD')
                if self.control_type=="angular_speed":
                    # cascade pid
                    roll_r = -0.54236707*(self.py-control_y)-0.42717549*(self.vy-0)
                    pitch_r= -0.54236707*(self.px-control_x)-0.42717549*(self.vx-0)

                    if pitch_r>angle_max:
                        pitch_r=angle_max
                    if pitch_r<-angle_max:
                        pitch_r=-angle_max
                    if roll_r>angle_max:
                        roll_r=angle_max
                    if roll_r<-angle_max:
                        roll_r=-angle_max
                    roll_rate=-5*(self.roll-roll_r)-0.1*self.roll_rate
                    pitch_rate=-5*(self.pitch-pitch_r)-0.1*self.pitch_rate
                    yaw_rate=-1*(self.yaw-yaw_r)-0.1*self.yaw_rate
                        
                    self.att.body_rate.x=roll_rate
                    self.att.body_rate.y=pitch_rate             
                    self.att.body_rate.z=yaw_rate
                    self.att.thrust = -0.5*(self.pz-control_z)-0.31774509*(self.vz-0)+0.55/math.cos(self.pitch)/math.cos(self.roll)#hover throttle is set to 0.55
                    self.att.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                    self.body_target_pub.publish(self.att)

                elif self.control_type=="angle":
                    ##PD control
                    roll_r = -0.54236707*(self.py-control_y)-0.42717549*(self.vy-0)
                    pitch_r= -0.54236707*(self.px-control_x)-0.42717549*(self.vx-0)
                    yaw_r=yaw_r
                    thrust = -0.5*(self.pz-control_z)-0.31774509*(self.vz-0)+0.55/math.cos(self.pitch)/math.cos(self.roll)#hover throttle is set to 0.55

                    if pitch_r>angle_max:
                        pitch_r=angle_max
                    if pitch_r<-angle_max:
                        pitch_r=-angle_max
                    if roll_r>angle_max:
                        roll_r=angle_max
                    if roll_r<-angle_max:
                        roll_r=-angle_max

                    q=self.euler_to_quaternion(roll_r,pitch_r,yaw_r)
                    self.att.orientation.w=q[0]
                    self.att.orientation.x=q[1]
                    self.att.orientation.y=q[2]
                    self.att.orientation.z=q[3]
                    self.att.thrust=thrust
                    self.att.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE+AttitudeTarget.IGNORE_YAW_RATE
                    self.body_target_pub.publish(self.att)

if __name__ == '__main__':
    con = pub(sys.argv[1], sys.argv[2])
    con.start()

