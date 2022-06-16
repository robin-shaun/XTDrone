#-*- coding: UTF-8 -*-
import rospy
import tty, termios
import sys,select
from mavros_msgs.msg import AttitudeTarget,PositionTarget,State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
import time
import PyKDL
import math
import numpy as np


class pub:
    def __init__(self):
        self.px = 0
        self.py = 0
        self.pz = 0
        self.vx,self.vy,self.vz = 0,0,0
        self.roll,self.pitch,self.yaw=0,0,0
        self.roll_rate,self.pitch_rate,self.yaw_rate=0,0,0
        self.arm_state = False
        self.mavros_state = State()
 
        #键盘控制说明
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
        p:pirnt key_contorl msg
        q: quit
         '''
        # subscribers
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("mavros/local_position/velocity_body",TwistStamped, self.local_vel_callback)
        self.odom_sub=rospy.Subscriber("mavros/local_position/odom",Odometry,self.odom_callback)
        self.mavros_sub = rospy.Subscriber("mavros/state", State, self.mavros_state_callback)

        # pub
        self.target_motion_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.body_target_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=2)
        # self.img_pub = rospy.Publisher('begin',Int32,queue_size=1)#话题
        self.flightModeService = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.armService = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    def local_pose_callback(self, msg):
        self.px = msg.pose.position.x
        self.py = -msg.pose.position.y#根据坐标系判断需不需要加负号
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
        rot = PyKDL.Rotation.Quaternion(x,y,z,w)#将四元素转化为欧拉角
        self.roll =rot.GetRPY()[0]#偏滚角，往右偏为正
        self.pitch =rot.GetRPY()[1]#俯仰角，往上为正
        self.yaw =rot.GetRPY()[2]#航向角，逆时针为正

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

    def euler_to_quaternion(self,roll, pitch, yaw):#将欧拉角转化为四元素
        ox=math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        oy=math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        oz=math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        ow=math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        return np.array([ow, ox, oy, oz])

    def getKey(self):#读取键盘
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
        contorl_flag=1#1是角速度，0是角度
        offborad_mission=None
        self.att =AttitudeTarget()
        target_raw_pose = PositionTarget()
        takeoff_x,takeoff_y,takeoff_z,takeoff_yaw=0,0,2,0
        control_x,control_y,control_z,yaw_r=1,0,2,0
        angle_max=0.3

        while 1:#主循环            
            self.time_now=time.time()
            self.clock=self.time_now-self.time_start
            key = self.getKey() 
            #键盘控制可以自己添加删除           
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
                self.arm()   #起飞前解锁       
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
            if self.mavros_state=='AUTO.LAND' and self.arm_state:
                print('AUTO.LAND ing') 
                if self.pz<0.1 and self.arm_state:
                    print('AUTO.LAND succeed')
                 
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
                if contorl_flag==1:#角速度+油门控制
                    #用串联pid
                    roll_r = -0.54236707*(self.py-control_y)-0.42717549*(self.vy-0)
                    pitch_r= -0.54236707*(self.px-control_x)-0.42717549*(self.vx-0)
                    #限制角度大小不大于angle_max弧度
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
                    yaw_rate=-1*(self.yaw-yaw_r)-0.1*self.yaw_rate#PD跟踪yaw_r
                    #这里可以再限制角速度
                    # if pitch_rate>angle_max:
                    #     pitch_rate=angle_max
                    self.att.body_rate.x=roll_rate
                    self.att.body_rate.y=pitch_rate             
                    self.att.body_rate.z=yaw_rate
                    self.att.thrust = -0.5*(self.pz-control_z)-0.31774509*(self.vz-0)+0.55/math.cos(self.pitch)/math.cos(self.roll)#此处认为0.55的油门可以使飞机悬停
                    self.att.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                    self.body_target_pub.publish(self.att)

                if contorl_flag==0:#角度+油门
                    ##PD控制，参数可以自己调整使用分段、模糊pid、串联pid等控制方式
                    roll_r = -0.54236707*(self.py-control_y)-0.42717549*(self.vy-0)
                    pitch_r= -0.54236707*(self.px-control_x)-0.42717549*(self.vx-0)
                    yaw_r=yaw_r
                    thrust = -0.5*(self.pz-control_z)-0.31774509*(self.vz-0)+0.55/math.cos(self.pitch)/math.cos(self.roll)#此处认为0.55的油门可以使飞机悬停
                    #限制角度大小不大于angle_max弧度
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
    con = pub()
    con.start()

