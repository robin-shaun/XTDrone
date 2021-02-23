#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Node():
    def __init__(self,linear_scaling,angular_scaling,keyboard=False):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.left_pub = None
        self.right_pub = None
        self.left_msg =None
        self.right_msg =None
        self.keyboard = keyboard

    def callback(self,data):
        rospy.logdebug("RX: Twist "+rospy.get_caller_id())
        rospy.logdebug("\tlinear:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.linear.x,
                                            data.linear.y,
                                            data.linear.z))
        rospy.logdebug("\tangular:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.angular.x,
                                            data.angular.y,
                                            data.angular.z))
        # scaling factors
        linfac = self.linear_scaling
        angfac = self.angular_scaling

        if self.keyboard:
            self.left_msg.data = data.linear.x
            self.right_msg.data = data.linear.x
            self.left_msg.data += -1*data.angular.z
            self.right_msg.data += data.angular.z
        else:
            self.left_msg.data = data.linear.x
            self.right_msg.data = data.angular.z

        rospy.logdebug("TX ")
        rospy.logdebug("\tleft:%f, right:%f"%(self.left_msg.data,
                                              self.right_msg.data))
        self.left_pub.publish(self.left_msg)
        self.right_pub.publish(self.right_msg)


if __name__ == '__main__':

    rospy.init_node('twist2drive', anonymous=True)

    # ROS Parameters
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',0.2)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',0.05)

    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))


    key = '--keyboard' in sys.argv
    node=Node(linear_scaling,angular_scaling,keyboard=key)

    # Publisher
    node.left_pub = rospy.Publisher("left_cmd",Float32,queue_size=10)
    node.right_pub = rospy.Publisher("right_cmd",Float32,queue_size=10)
    node.left_msg = Float32()
    node.right_msg = Float32()

    # Subscriber
    rospy.Subscriber("cmd_vel",Twist,node.callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
