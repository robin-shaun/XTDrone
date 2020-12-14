#!/usr/bin/env python
import roslib
import rospy
# import socket

# UDP_IP = "127.0.0.1"
# UDP_PORT = 5005
# MESSAGE = ""

# print "UDP target IP:", UDP_IP
# print "UDP target port:", UDP_PORT
# print "message:", MESSAGE

# sock = socket.socket(socket.AF_INET, # Internet
#              socket.SOCK_DGRAM) # UDP


from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
pubt = rospy.Publisher('/colorado/Driving/Throttle', Float64, queue_size=10)
pubs = rospy.Publisher('/colorado/Driving/Steering', Float64, queue_size=10)
Throttle=0
Steer=0
rospy.init_node('cmd_vel_listener')
rate = rospy.Rate(30)
def callback(msg):

    Throttle = msg.linear.x
    Steer = msg.angular.z
    # rospy.loginfo(Throttle)
    pubt.publish(Float64(Throttle))
    pubs.publish(Float64(-Steer))
    # MESSAGE ="%f %f" % (Throttle, -Steer)
    # sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

def listener():
    
    rospy.Subscriber("/xtdrone/rover_0/cmd_vel_flu", Twist, callback)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
