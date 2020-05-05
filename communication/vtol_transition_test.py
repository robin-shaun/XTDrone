import rospy
import tf
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandVtolTransition
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import time
from pyquaternion import Quaternion
import math
import sys


trans = rospy.ServiceProxy('/mavros/cmd/vtol_transition', CommandVtolTransition)
print(trans(state = 3))
