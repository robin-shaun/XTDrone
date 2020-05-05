import rospy
import tf
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import time
from pyquaternion import Quaternion
import math
import sys
from px4_communication import PX4Communication
px4_com = PX4Communication()
rospy.init_node("px4_communication")
rate = rospy.Rate(100)
cnt = 0
while(rospy.is_shutdown):
    px4_com.pose_target_pub.publish(px4_com.target_pose)
    if cnt == 1000:
        px4_com.target_pose.pose.position.x = 10