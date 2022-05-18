#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

## Gazebo test utility
##        check a link's pose against some reference pose
##        good for unit testing proper behavior

PKG = 'gazebo_plugins'
NAME = 'test_link_pose'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from nav_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf import transformations

class LinkPoseTest(unittest.TestCase):
    def __init__(self, *args):
        super(LinkPoseTest, self).__init__(*args)
        self.success = False

        self.link_error_total = 0
        self.link_error_rms = 0
        self.link_sample_count = 0
        self.valid_sample_count = 0
        self.link_error_max = 0

        self.min_link_samples_topic     = "~min_link_samples"
        self.min_link_samples     = 1000

        self.min_valid_samples_topic     = "~min_valid_samples"
        self.min_valid_samples     = 0

        self.tolerance_topic = "~error_tolerance"
        self.tolerance = 0.01

        self.max_error_topic = "~max_error"
        self.max_error = 0.02

        # in seconds
        self.test_duration_topic = "~test_duration"
        self.test_duration = 10.0

        # test start time in seconds
        self.test_start_time_topic = "~test_start_time"
        self.test_start_time = 0.0

        self.link_pose_topic_name = "~link_pose_topic_name"
        self.link_pose_topic = "/model_1/link_2/pose"

        self.valid_pose_topic_name = "~valid_pose_topic_name"
        self.valid_pose_topic = "/p3d_valid"

        self.link_pose = Pose()
        self.valid_pose = Pose()

    def printLinkPose(self, p3d):
        print "P3D pose translan: " + "x: " + str(p3d.pose.pose.position.x)
        print "                   " + "y: " + str(p3d.pose.pose.position.y)
        print "                   " + "z: " + str(p3d.pose.pose.position.z)
        print "P3D pose rotation: " + "x: " + str(p3d.pose.pose.orientation.x)
        print "                   " + "y: " + str(p3d.pose.pose.orientation.y)
        print "                   " + "z: " + str(p3d.pose.pose.orientation.z)
        print "                   " + "w: " + str(p3d.pose.pose.orientation.w)
        print "P3D rate translan: " + "x: " + str(p3d.twist.twist.linear.x)
        print "                   " + "y: " + str(p3d.twist.twist.linear.y)
        print "                   " + "z: " + str(p3d.twist.twist.linear.z)
        print "P3D rate rotation: " + "x: " + str(p3d.twist.twist.angular.x)
        print "                   " + "y: " + str(p3d.twist.twist.angular.y)
        print "                   " + "z: " + str(p3d.twist.twist.angular.z)


    def linkP3dInput(self, p3d):
        #self.printLinkPose(p3d)
        self.link_pose = p3d.pose.pose

        # start logging after we get minimum valid pose specified
        # todo: synchronize the two input streams somehow
        if self.valid_sample_count >= self.min_valid_samples:
          self.link_sample_count += 1
          tmpx = self.link_pose.position.x - self.valid_pose.position.x
          tmpy = self.link_pose.position.y - self.valid_pose.position.y
          tmpz = self.link_pose.position.z - self.valid_pose.position.z
          error = math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz)
          self.link_error_total += error
          self.link_error_rms = self.link_error_total / self.link_sample_count
          if error > self.link_error_max:
              self.link_error_max =  math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz)

    def validP3dInput(self, p3d):
        self.valid_pose = p3d.pose.pose
        self.valid_sample_count += 1

    def checkPose(self):
        # difference in pose
        print  "error: "   + str(self.link_sample_count) \
                           + " error:" + str(self.link_error_total) \
                           + " avg err:" + str(self.link_error_rms) \
                           + " max err:" + str(self.link_error_max) \
                           + " count: " + str(self.link_sample_count)
        if self.link_sample_count >= self.min_link_samples:
          if self.link_error_rms < self.tolerance:
            if self.link_error_max < self.max_error:
              self.success = True


    def test_link_pose(self):
        print "LNK\n"
        rospy.init_node(NAME, anonymous=True)
        self.link_pose_topic = rospy.get_param(self.link_pose_topic_name,self.link_pose_topic);
        self.valid_pose_topic = rospy.get_param(self.valid_pose_topic_name,self.valid_pose_topic);
        self.min_link_samples = rospy.get_param(self.min_link_samples_topic,self.min_link_samples);
        self.min_valid_samples = rospy.get_param(self.min_valid_samples_topic,self.min_valid_samples);
        self.tolerance = rospy.get_param(self.tolerance_topic,self.tolerance);
        self.max_error = rospy.get_param(self.max_error_topic,self.max_error);
        self.test_duration = rospy.get_param(self.test_duration_topic,self.test_duration);
        self.test_start_time = rospy.get_param(self.test_start_time_topic,self.test_start_time);

        while not rospy.is_shutdown() and time.time() < self.test_start_time:
            rospy.stdinfo("Waiting for test to start at time [%s]"% self.test_start_time)
            time.sleep(0.1)

        print "subscribe"
        rospy.Subscriber(self.link_pose_topic, Odometry, self.linkP3dInput)
        rospy.Subscriber(self.valid_pose_topic, Odometry, self.validP3dInput)

        start_t = time.time()
        timeout_t = start_t + self.test_duration
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.checkPose()
            time.sleep(0.1)
        self.assert_(self.success)

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, sys.argv[0], LinkPoseTest, sys.argv) #, text_mode=True)


