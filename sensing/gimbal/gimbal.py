#!/usr/bin/env python

# gimbal.py -- control the gimbal of a uav
# andy March/2023

import sys

import rospy
from mavros_msgs.msg import MountControl

class GimbolCtrl:
	def __init__(self, type, id):
		self.type = type
		self.id = id

		# parameters
		self.gimbal_pitch_ = -60.0
		self.gimbal_yaw_ = 0.0
		self.gimbal_roll_ = 0.0

		# Node init
		rospy.init_node("gimbal_control_%s_%s" % (type, id))

		# Publisher
		self.mount_ctrl_pub = rospy.Publisher("/%s_%s/mavros/mount_control/command"%(self.type, self.id),\
			MountControl, queue_size=1)

		# Rate
		self.rate = rospy.Rate(30)

		# start
		self.start()

	def start(self):
		while not rospy.is_shutdown():
			# change
			self.gimbal_yaw_ += 1


			# generate mountctrl msg and publish
			msg = MountControl()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = "map"
			msg.mode = 2
			msg.pitch = self.gimbal_pitch_
			msg.roll = self.gimbal_roll_
			msg.yaw = self.gimbal_yaw_
			self.mount_ctrl_pub.publish(msg)

			# spin()
			self.rate.sleep()





if __name__ == "__main__":
	uav_type = sys.argv[1]
	uav_id = sys.argv[2]

	g_ctrl = GimbolCtrl(uav_type, uav_id)
