import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import sys

Kp = 0.002
Vx = 5

def lane_mid_error_callback(msg):
    global twist
    if msg.data == 1000:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    else:
        if abs(msg.data) > 20:
            twist.angular.z = - Kp * msg.data
        else:
            twist.angular.z = 0.0
            
        twist.linear.x = Vx * (1 - abs(twist.angular.z))


if __name__ == "__main__":
    ugv_num = sys.argv[1]
    rospy.init_node('ugv_self_driving_'+ugv_num)
    cmd_vel_flu_pub = rospy.Publisher('/ugv_'+ugv_num+'/cmd_vel', Twist, queue_size=2)
    lane_mid_error_sub = rospy.Subscriber("/ugv_"+ugv_num+"/lane_mid_error",Int16,callback=lane_mid_error_callback)                                   
    twist = Twist()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        cmd_vel_flu_pub.publish(twist)
        rate.sleep()
                                        
    

