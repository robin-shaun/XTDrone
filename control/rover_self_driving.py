import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

Kp = 0.003
Vx = 3

def lane_mid_error_callback(msg):
    global twist
    if msg.data == 1000:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
    else:
        if abs(msg.data) > 20:
            twist.linear.x = Kp * msg.data
        else:
            twist.linear.x = 0.0
        twist.linear.y = Vx * (1 - twist.linear.x)


if __name__ == "__main__":
    rospy.init_node('rover_self_driving')
    cmd_vel_flu_pub = rospy.Publisher('/xtdrone/rover_0/cmd_vel_flu', Twist, queue_size=2)
    lane_mid_error_sub = rospy.Subscriber("/rover_0/lane_mid_error",Int16,callback=lane_mid_error_callback)                                   
    twist = Twist()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        cmd_vel_flu_pub.publish(twist)
        rate.sleep()
                                        
    

