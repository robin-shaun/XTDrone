import rospy
from geometry_msgs.msg import Vector3, PoseStamped
import time
import numpy 
import sys

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
pose = [None]*vehicle_num
pose_sub = [None]*vehicle_num
avoid_vel_pub = [None]*vehicle_num
avoid_radius = 1.5
aid_vec1 = [1, 0, 0]
aid_vec2 = [0, 1, 0]
vehicles_avoid_vel = [Vector3()] * vehicle_num

def pose_callback(msg, id):
    pose[id] = msg

rospy.init_node('avoid')
for i in range(vehicle_num):
    pose_sub[i] = rospy.Subscriber(vehicle_type+'_'+str(i)+'/mavros/local_position/pose',PoseStamped,pose_callback,i,queue_size=1)
    avoid_vel_pub[i] = rospy.Publisher("/xtdrone/"+vehicle_type+'_'+str(i)+"/avoid_vel", Vector3,queue_size=1)

time.sleep(1)
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    for i in range(vehicle_num):
        position1 = numpy.array([pose[i].pose.position.x, pose[i].pose.position.y, pose[i].pose.position.z])
        for j in range(1, vehicle_num-i):
            position2 = numpy.array([pose[i+j].pose.position.x, pose[i+j].pose.position.y, pose[i+j].pose.position.z])
            dir_vec = position1-position2
            if numpy.linalg.norm(dir_vec) < avoid_radius:
                cos1 = dir_vec.dot(aid_vec1)/(numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec1))
                cos2 = dir_vec.dot(aid_vec2)/(numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec2))
                if  abs(cos1) < abs(cos2):
                    avoid_vel = numpy.cross(dir_vec, aid_vec1)/numpy.linalg.norm(numpy.cross(dir_vec, aid_vec1))
                else:
                    avoid_vel = numpy.cross(dir_vec, aid_vec2)/numpy.linalg.norm(numpy.cross(dir_vec, aid_vec2))
                if dir_vec[2] * avoid_vel[2] > 0:
                    vehicles_avoid_vel[i] = Vector3(vehicles_avoid_vel[i].x+avoid_vel[0],vehicles_avoid_vel[i].y+avoid_vel[1],vehicles_avoid_vel[i].z+avoid_vel[2])
                    vehicles_avoid_vel[i+j] = Vector3(vehicles_avoid_vel[i+j].x-avoid_vel[0],vehicles_avoid_vel[i+j].y-avoid_vel[1],vehicles_avoid_vel[i+j].z-avoid_vel[2])
                else:
                    vehicles_avoid_vel[i] = Vector3(vehicles_avoid_vel[i].x-avoid_vel[0],vehicles_avoid_vel[i].y-avoid_vel[1],vehicles_avoid_vel[i].z-avoid_vel[2])
                    vehicles_avoid_vel[i+j] = Vector3(vehicles_avoid_vel[i+j].x+avoid_vel[0],vehicles_avoid_vel[i+j].y+avoid_vel[1],vehicles_avoid_vel[i+j].z+avoid_vel[2])
    for i in range(vehicle_num):
        avoid_vel_pub[i].publish(vehicles_avoid_vel[i])
    vehicles_avoid_vel = [Vector3()] * vehicle_num
    rate.sleep()
            
            
