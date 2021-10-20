#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from pyquaternion import Quaternion
import time
import sys
import moveit_commander
# from InverseKinematics import IK
from kinematics import kinematic_analysis
import math

def local_pose_callback(data):
    global local_pose, z_cnt 
    # if abs(local_pose.pose.position.z - data.pose.position.z) < 0.005:
    #     z_cnt = z_cnt + 1
    # else:
    #     z_cnt = 0
    local_pose = data

if __name__ == '__main__':
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node(vehicle_type+'_'+vehicle_id+'_arm_control')
    # ik = IK('arm')
    # ik.setLinkLength(L1=ik.l1 + 0.89, L4=ik.l4 - 0.3)
    # print('link length: ', ik.getLinkLength())
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander('arm')
    gripper = moveit_commander.MoveGroupCommander('gripper')
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    gripper.set_goal_joint_tolerance(0.001)
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(1)
    arm.set_max_velocity_scaling_factor(1)
    gripper.set_max_acceleration_scaling_factor(1)
    gripper.set_max_velocity_scaling_factor(1)
    # 控制机械臂先回到特定位置
    joint_positions = [0, 0, 0.785, 0, 0]
    #joint_positions = [0, 0.785, 0.785, 0, 0]
    arm.set_joint_value_target(joint_positions)
    arm.go()
    rospy.sleep(1)

    #控制机械抓张开大角度
    grasp_angle = 0.7
    gripper.set_joint_value_target([grasp_angle,-grasp_angle,grasp_angle,-grasp_angle,grasp_angle,-grasp_angle])
    gripper.go()
    rospy.sleep(1)

    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()

    # 设置目标位置所使用的参考坐标系
    reference_frame = 'arm_base_link'
    arm.set_pose_reference_frame(reference_frame)

    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)

    # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
    # 姿态使用四元数描述，基于base_link坐标系
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = 0.0636649477824
    target_pose.pose.position.y = -0.00552153656686
    target_pose.pose.position.z = 0.26141858437
    target_pose.pose.orientation.x = 0.70692
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.70729

    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()
    
    # 设置机械臂终端运动的目标位姿
    arm.set_pose_target(target_pose, end_effector_link)
    
    # 规划运动路径
    traj = arm.plan()
    
    # 按照规划的运动路径控制机械臂运动
    arm.execute(traj)
    rospy.sleep(1)

    cmd_vel_enu = Twist()   
    local_pose = PoseStamped()
    time_cnt = 0
    z_cnt = 0
    cmd_vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=2)
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/vision_pose/pose", PoseStamped, local_pose_callback)
    tfBuffer = Buffer()
    tflistener = TransformListener(tfBuffer)
    rate = rospy.Rate(1)

    Kp_xy = 2
    land_vel = 0.1

    arm_x_bias = 0
    arm_y_bias = 0
    arm_z_bias = 0
    
    while not rospy.is_shutdown():
        try:
        #     # tfstamped = tfBuffer.lookup_transform('arm_base_link', 'tag_0', rospy.Time(0))
            l0 = tfBuffer.lookup_transform('arm_base_link', 'wrist_1_link', rospy.Time(0))
        #     # l1 = tfBuffer.lookup_transform('shoulder_link', 'upper_arm_link', rospy.Time(0))
        #     # l2 = tfBuffer.lookup_transform('upper_arm_link', 'forearm_link', rospy.Time(0))
        #     # l3 = tfBuffer.lookup_transform('forearm_link', 'wrist_1_link', rospy.Time(0))
            print(l0.transform.translation.x)
            print(l0.transform.translation.y)
            print(l0.transform.translation.z)
        #     # print(math.sqrt(l0.transform.translation.x**2+l0.transform.translation.y**2+l0.transform.translation.z**2))
        #     # print(math.sqrt(l1.transform.translation.x**2+l1.transform.translation.y**2+l1.transform.translation.z**2))
        #     # print(math.sqrt(l2.transform.translation.x**2+l2.transform.translation.y**2+l2.transform.translation.z**2))
        #     # print(math.sqrt(l3.transform.translation.x**2+l3.transform.translation.y**2+l3.transform.translation.z**2))
        #     # print('******')
        #     continue
        except:
            rate.sleep()
            continue
        # if z_cnt < 20:
        #     cmd_vel_enu.linear.x = Kp_xy * (tfstamped.transform.translation.y)
        #     cmd_vel_enu.linear.y = - Kp_xy* (tfstamped.transform.translation.z)
        #     cmd_vel_enu.linear.z  =  - land_vel
        #     cmd_vel_pub.publish(cmd_vel_enu)
        # else:
        # target_x = (tfstamped.transform.translation.z + arm_x_bias) * 100
        # target_y = (-tfstamped.transform.translation.y + arm_y_bias) * 100
        # target_z = (tfstamped.transform.translation.x + arm_z_bias) * 100
        # print((target_x, target_y, target_z))
        target_x = 1250
        target_y = 0
        target_z = 1500
        # theta3, theta4, theta5, theta6 = ik.getRotationAngle((target_x, target_y, target_z), 90)
        # theta3, theta4, theta5, theta6 = ik.getRotationAngle((0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4), 90)
        alpha_list = []
        for alp in range(90, -90, -1):#遍历爪子与水平面的夹角，在-25到-65求解，其他范围夹取物体效果不好
            if kinematic_analysis(target_x, target_y, target_z, alp):
                alpha_list.append(alp)
        if len(alpha_list) > 0:
            if target_y > 2150:
                best_alpha = max(alpha_list)
            else:
                best_alpha = min(alpha_list)
        theta3, theta4, theta5, theta6 = kinematic_analysis(target_x, target_y, target_z, best_alpha)
        print(theta3, theta4, theta5, theta6)
        joint_positions = [theta6/180*3.14, theta5/180*3.14, theta4/180*3.14, -theta3/180*3.14, 0]
        joint_positions[1] = joint_positions[1] - 0.785
        joint_positions[2] = joint_positions[2] + 0.785
        print(joint_positions)
        arm.set_joint_value_target(joint_positions)
        # 控制机械臂完成运动
        arm.go()
        rate.sleep()