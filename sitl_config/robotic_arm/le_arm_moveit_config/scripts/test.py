import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, PoseStamped
import inspect
import ctypes

cmd = String()
twist = Twist()

rospy.init_node('iris_auto_control')
multi_cmd_vel_flu_pub = rospy.Publisher('/xtdrone/' + 'iris_0' + '/cmd_vel_flu', Twist, queue_size=1)
multi_cmd_pub = rospy.Publisher('/xtdrone/' + 'iris_0' + '/cmd', String,queue_size=3)


twist.linear.x = 0.0
twist.linear.y = 0.0
twist.linear.z = 0.32
twist.angular.x = 0.0
twist.angular.y = 0.0
twist.angular.z = 0.0
rospy.loginfo("process 1")

cmd=''
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
time.sleep(1)
rospy.loginfo("process 2")

print('take off')
cmd='OFFBOARD'
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
time.sleep(1)
rospy.loginfo("process 3")

cmd = 'ARM'
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 4")

time.sleep(8)

print('hover')
cmd = 'HOVER'
twist.linear.z = 0.0
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 5")
time.sleep(1)

print('move')
cmd=''
twist.linear.x = 0.20
twist.linear.y = 0.1
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 6")
time.sleep(6)

print('hover')
cmd = 'HOVER'
twist.linear.x = 0.0
twist.linear.y = 0.0
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 7")
time.sleep(20)
rospy.loginfo("process 7")

print('move')
cmd=''
twist.linear.x = -0.20
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 8")
time.sleep(7)

print('hover')
cmd = 'HOVER'
twist.linear.x = 0.0
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 9")
time.sleep(1)

print('landing')
cmd = 'AUTO.LAND'
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 10")
time.sleep(10)

cmd = 'DISARM'
multi_cmd_vel_flu_pub.publish(twist)
multi_cmd_pub.publish(cmd)
rospy.loginfo("process 11")
