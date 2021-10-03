# -*- encoding: utf-8 -*-
import socket
import time
import kinematics
import LeArm
import sys
IP = '192.168.149.2' #填写服务器端的IP地址
port = 40005 #端口号必须一致
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
last_target_x = 0
last_target_y = 0
last_target_z = 0
try:
    s.connect((IP,port))
except Exception as e:
    print('server not find or not open')
    sys.exit(0)
LeArm.setServo(1, 500, 20)
LeArm.setServo(3, 800, 20)
LeArm.setServo(4, 900, 20)
LeArm.setServo(5, 800, 20)
while True:
    trigger = "Ready"
    s.sendall(trigger.encode())
    data = s.recv(1024)
    data = data.decode()
    print('recieved:',data)
    target = data.split(',')
    target_x = int(float(target[0]))
    target_y = int(float(target[1]))
    target_z = int(float(target[2]))
    if (target_x-last_target_x)**2+(target_y-last_target_y)**2+(target_z-last_target_z)**2 < 90000:
        continue
    LeArm.setServo(1, 500, 20)
    LeArm.setServo(3, 800, 20)
    LeArm.setServo(4, 900, 20)
    LeArm.setServo(5, 800, 20)
    time.sleep(0.33)
    kinematics.ki_move(target_x,target_y,target_z,20)
    time.sleep(0.33)
    LeArm.setServo(1, 1300, 20)
    time.sleep(0.5)
    last_target_x = target_x
    last_target_y = target_y
    last_target_z = target_z
s.close()

