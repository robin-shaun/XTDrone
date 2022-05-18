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

## Gazebo test bumper for a box

PKG = 'gazebo_plugins'
NAME = 'test_bumper'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from nav_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import ContactState

class BumperTest(unittest.TestCase):
    def __init__(self, *args):
        super(BumperTest, self).__init__(*args)
        self.success = False

        self.sample_count = 0

        self.min_samples_topic     = "~min_samples"
        self.min_samples     = 1000

        # in seconds
        self.test_duration_topic = "~test_duration"
        self.test_duration = 10.0

        # test start time in seconds
        self.test_start_time_topic = "~test_start_time"
        self.test_start_time = 0.0

        self.bumper_topic_name = "~bumper_topic_name"
        self.bumper_topic = "/test_bumper"
        self.bumper_state = ContactsState()

        self.fz_sum = 0
        self.fz_avg = 0

    def bumperStateInput(self, contacts_state):
        self.bumper_state = contacts_state
        self.sample_count+=1
        self.fz_sum += contacts_state.states[0].total_wrench.force.z
        self.fz_avg = self.fz_sum / self.sample_count

    def checkContact(self):
        # see if total wrench is close to 98.1N
        if self.sample_count > 20:
          if abs(self.fz_avg - 98.1) < 0.01:
            print "z force ",self.fz_avg
            self.success = True


    def test_bumper(self):
        print "LNK\n"
        rospy.init_node(NAME, anonymous=True)
        self.bumper_topic = rospy.get_param(self.bumper_topic_name,self.bumper_topic);
        self.min_samples = rospy.get_param(self.min_samples_topic,self.min_samples);
        self.test_duration = rospy.get_param(self.test_duration_topic,self.test_duration);
        self.test_start_time = rospy.get_param(self.test_start_time_topic,self.test_start_time);

        while not rospy.is_shutdown() and time.time() < self.test_start_time:
            rospy.stdinfo("Waiting for test to start at time [%s]"% self.test_start_time)
            time.sleep(0.1)

        print "subscribe"
        rospy.Subscriber(self.bumper_topic, ContactsState, self.bumperStateInput)

        start_t = time.time()
        timeout_t = start_t + self.test_duration
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.checkContact()
            time.sleep(0.1)
        self.assert_(self.success)

if __name__ == '__main__':
    print "Waiting for test to start at time "
    rostest.run(PKG, sys.argv[0], BumperTest, sys.argv) #, text_mode=True)


