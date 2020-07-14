import rospy
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import SetMode
# Single vehicle Mount Command
rospy.init_node('gimbal_control')
mountCnt = rospy.Publisher('/typhoon_h480_0/mavros/mount_control/command', MountControl, queue_size=10)
FlightMode = rospy.ServiceProxy("/typhoon_h480_0/mavros/set_mode", SetMode)
i=0
rate=rospy.Rate(100)
gimbal_pitch_ = -30
gimbal_yaw_ = 0.0
gimbal_roll_ = 0.0
FlightMode(custom_mode='offboard')
while not rospy.is_shutdown():
    msg = MountControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.mode = 2
    #if i%20 == 0:
    	#gimbal_yaw_ += 10
    	#print("yaw increased to", gimbal_yaw_)
    msg.pitch = gimbal_pitch_
    msg.roll = gimbal_roll_
    msg.yaw = gimbal_yaw_
    mountCnt.publish(msg)
    i+=1
    rate.sleep()