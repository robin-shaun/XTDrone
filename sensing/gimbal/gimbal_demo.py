import rospy
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import MountConfigure

import std_msgs.msg
# Single vehicle Mount Command
rospy.init_node('gimbal_control')
mountCnt = rospy.Publisher('/mavros/mount_control/command', MountControl, queue_size=10)
mountConfig = rospy.ServiceProxy("/mavros/mount_control/configure", MountConfigure)
i=0
rate=rospy.Rate(100)
gimbal_pitch_ = -70
gimbal_yaw_ = 0.0
gimbal_roll_ = 0.0
srvheader=std_msgs.msg.Header()
srvheader.stamp=rospy.Time.now()
srvheader.frame_id="map"

mountConfig(header=srvheader,mode=2,stabilize_roll=0,stabilize_yaw=1,stabilize_pitch=0)
while not rospy.is_shutdown():
    msg = MountControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.mode = 2
    if i%20 == 0:
    	gimbal_yaw_ += 0
    	print("yaw increased to", gimbal_yaw_)
    msg.pitch = gimbal_pitch_
    msg.roll = gimbal_roll_
    msg.yaw = gimbal_yaw_
    mountCnt.publish(msg)
    i+=1
    rate.sleep()
