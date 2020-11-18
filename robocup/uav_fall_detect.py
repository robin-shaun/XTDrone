import rospy
from mavros_msgs.msg import State
import sys

def state_callback(msg):
    print(msg.armed)

if __name__ == "__main__":
    rospy.init_node('uav_fall_detect')
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    state_sub = rospy.Subscriber('/'+vehicle_type+'_'+vehicle_id+'/mavros/state', State, state_callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


