#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys                                                                   
import signal
import threading
import numpy as np
import rospy
import moveit_commander
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
import tf2_ros
from tf2_geometry_msgs import PoseStamped,PointStamped


RADIAN = 57.3


class Processer:
    def __init__(self):
        self.apriltag_list = []    
        self.apriltag_id = None    
        self.apriltag_x = None
        self.apriltag_y = None
        self.apriltag_z = None

        rospy.init_node('test_apriltag', anonymous=True)
        
    def quit(self,signum,frame):
        '''
        用于ctrl+c退出
        '''        
        print('')
        print('stop fusion')
        sys.exit()   
        
    def apriltag_callback(self,data):
        '''
        apriltag的callback函数
        '''
        if data.detections:
            self.apriltag_list = []
            temp_dict = {}
            for d in data.detections:
                id = d.id[0]
                pos_x = d.pose.pose.pose.position.x
                pos_y = d.pose.pose.pose.position.y
                pos_z = d.pose.pose.pose.position.z
                ori_x = d.pose.pose.pose.orientation.x 
                ori_y = d.pose.pose.pose.orientation.y 
                ori_z = d.pose.pose.pose.orientation.z 
                ori_w = d.pose.pose.pose.orientation.w 
                
                temp_dict['id'] = id
                temp_dict['pos'] = [pos_x,pos_y,pos_z] 
                temp_dict['ori'] = [ori_x,ori_y,ori_z,ori_w]                
                self.apriltag_list.append(temp_dict)
            #print(self.apriltag_list)   
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.apriltag_list)
        else:
            self.apriltag_list = []
            #print(self.apriltag_list)
            
    def apriltag_listener(self):
        '''
        侦听apriltag
        '''
        thread_spin_1 = threading.Thread(target=self.thread_spin_job)
        thread_spin_1.start()    
                
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.apriltag_callback)    
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

    def gripper_fk(self,action):
        '''
        正向运动学，控制夹爪
        '''
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)        
        # 初始化需要使用move group控制的机械臂中的arm group
        gripper = moveit_commander.MoveGroupCommander('gripper')
        # 设置机械臂运动的允许误差值，单位弧度
        gripper.set_goal_joint_tolerance(0.001)
 
        # 设置允许的最大速度和加速度，范围0~1
        gripper.set_max_acceleration_scaling_factor(0.5)
        gripper.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置
        gripper.set_named_target(action)
        gripper.go()
        rospy.sleep(1)

    def arm_fk(self,action):
        '''
        正向运动学，控制机械臂
        '''
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        # 设置机械臂运动的允许误差值，单位弧度
        arm.set_goal_joint_tolerance(0.001)
 
        # 设置允许的最大速度和加速度，范围0~1
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target(action)
        arm.go()
        rospy.sleep(1)      
        
    def arm_ik(self,x,y,z):
        '''
        逆向运动学，控制机械臂先回到初始化位置
        '''
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
            
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
  
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'arm_base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.5)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        target_pose.pose.orientation.x = 0.700952
        target_pose.pose.orientation.y = 0.79637
        target_pose.pose.orientation.z = -0.10829
        target_pose.pose.orientation.w = -0.109579

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        #arm.set_pose_target(target_pose, end_effector_link)
        arm.set_joint_value_target(target_pose, end_effector_link, True)
        
        # 规划运动路径
        traj = arm.plan()

        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)        
    
    def move_shoulder_pan(self,y_delta):
        '''
        正向运动学，控制机械臂
        先获取机械臂关节位置，然后修改shoulder关节值
        '''
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # 获取机械臂当前位置
        arm_joint_values = arm.get_current_joint_values()
        #print(arm_joint_values)
        
        # 修改shoulder位置   
        arm_joint_values[0] = arm_joint_values[0] + y_delta
        
        arm.set_joint_value_target(arm_joint_values)
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)      

    def calc_radian(self,x,y):
        '''用反三角函数计算夹角角度，然后再换算成弧度'''
        int = np.arctan(y/x)
        angel = np.degrees(int)
        radian = angel / RADIAN
        #print(radian)
        return radian

    def tf_to_world(self, frame_id, tag):
        '''
        相机坐标系转世界坐标系
        '''
        #设置tf2_listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        #定义要转换的输入
        p = PoseStamped()
        p.header.stamp = rospy.Time()
        p.header.frame_id = frame_id
        p.pose.position.x = tag['pos'][0]
        p.pose.position.y = tag['pos'][1]
        p.pose.position.z = tag['pos'][2]
        p.pose.orientation.x = tag['ori'][0]
        p.pose.orientation.y = tag['ori'][1]
        p.pose.orientation.z = tag['ori'][2]
        p.pose.orientation.w = tag['ori'][3]
     
        res=tf_buffer.transform(p,'world',timeout=rospy.Duration(5))       
        #print(res)    
        return res

    def thread_spin_job(self):
        '''
        spin用于多线程中
        '''
        rospy.spin()
        
        
if __name__ == '__main__':
    # 实例化进程
    proc = Processer() 
 
    # 机械臂复位到观察位置
    proc.arm_fk('arm_observe_pose')
        
    # 开始侦听apriltag
    proc.apriltag_listener()
    time.sleep(2)  # 延迟2s,等待进程获取到apriltag的值
        
    while not rospy.is_shutdown():
        # 用于控制ctrl+c退出
        signal.signal(signal.SIGINT, proc.quit)                   
        signal.signal(signal.SIGTERM, proc.quit)   
    
        if len(proc.apriltag_list):
            # 获取第一个apriltag_list中的第一个元素
            first_tag = proc.apriltag_list[0]
            
            # 计算得到apriltag_list中第一个元素的世界坐标系
            tag_world = proc.tf_to_world('camera_link_new',first_tag) 
            tag_x = tag_world.pose.position.x
            tag_y = tag_world.pose.position.y
            tag_z = tag_world.pose.position.z     

            # 计算得到shoulder需要旋转的弧度        
            radian_delta = proc.calc_radian(tag_x,tag_y)
            
            # 移动shoulder到apriltag_list中第一个元素的正前方     
            proc.move_shoulder_pan(radian_delta)

            # 打开夹爪
            # proc.gripper_fk('gripper_open_pose')
            
            # 逆向运动抓取
            print([tag_x,tag_y,tag_z])
            proc.arm_ik(tag_x,tag_y,tag_z)

            # 合上夹爪
            proc.gripper_fk('gripper_close_pose')    

            # 机械臂复位到观察位置
            proc.arm_fk('arm_observe_pose')
        