#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from pyquaternion import Quaternion
import socket           
import time
import sys

def local_pose_callback(data):
    global local_pose, z_cnt 
    if abs(local_pose.pose.position.z - data.pose.position.z) < 0.005:
        z_cnt = z_cnt + 1
    else:
        z_cnt = 0
    local_pose = data

if __name__ == '__main__':
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node(vehicle_type+'_'+vehicle_id+'_arm_control')
    cmd_vel_enu = Twist()   
    local_pose = PoseStamped()
    time_cnt = 0
    z_cnt = 0
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/vision_pose/pose", PoseStamped, local_pose_callback)
    cmd_vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=2)
    tfBuffer = Buffer()
    tflistener = TransformListener(tfBuffer)
    rate = rospy.Rate(20)

    IP = "192.168.149.2"
    port = 40005
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((IP,port))
    s.listen(1)
    print('listen at port :',port)
    conn,addr = s.accept()
    print('connected by',addr)


    Kp_xy = 0.5
    land_vel = 0.3

    arm_x_bias = -0.0434
    arm_y_bias = 0.2155
    arm_z_bias = 0.02
    
    while not rospy.is_shutdown():
        try:
            tfstamped = tfBuffer.lookup_transform('camera_link', 'tag_0', rospy.Time(0))
        except:
            try:
                tfstamped = tfBuffer.lookup_transform('camera_link', 'tag_1', rospy.Time(0))
            except:
                try:
                    tfstamped = tfBuffer.lookup_transform('camera_link', 'tag_2', rospy.Time(0))
                except:
                    try:
                        tfstamped = tfBuffer.lookup_transform('camera_link', 'tag_3', rospy.Time(0))
                    except:
                        try:
                            tfstamped = tfBuffer.lookup_transform('camera_link', 'tag_4', rospy.Time(0))
                        except:
                            try:
                                tfstamped = tfBuffer.lookup_transform('camera_link', 'tag_5', rospy.Time(0))
                            except:
                                 rate.sleep()
                                 continue
        # print(z_cnt)
        if z_cnt < 20:
            cmd_vel_enu.linear.x = Kp_xy * (tfstamped.transform.translation.y)
            cmd_vel_enu.linear.y = Kp_xy* (tfstamped.transform.translation.x)
            cmd_vel_enu.linear.z  =  - land_vel
            cmd_vel_pub.publish(cmd_vel_enu)
        else:
            data = conn.recv(1024)
            data = data.decode()
            if not data:
                continue
            target_x = (tfstamped.transform.translation.x + arm_x_bias) * 1e4 
            target_y = (tfstamped.transform.translation.y + arm_y_bias) * 1e4 
            target_z = (tfstamped.transform.translation.z + arm_z_bias) * 1e4
            if data == 'Ready':
                print('recieved message:',data)
                send =str(target_x) +',' + str(target_y) + ',' + str(target_z)
                conn.sendall(send.encode())
                time.sleep(1)
            data = 'Not ready'
            rate.sleep()

    conn.close()
    s.close()