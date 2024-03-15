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

## Gazebo send wrench topic for gazebo_ros_force consumption

PKG = 'gazebo_plugins'
NAME = 'set_wrench'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest

from geometry_msgs.msg import Wrench, Vector3
import tf.transformations as tft
from numpy import float64

class SimIfaceControl():
  def __init__(self):
    self.update_rate=10
    self.target_l = [0,0,0] # linear
    self.target_e = [0,0,0] # angular
    self.topic_name = "set_wrench_topic"
    self.timeout=1
    rospy.init_node(NAME, anonymous=True)

  def setWrench(self):
    # get goal from commandline
    for i in range(0,len(sys.argv)):
      if sys.argv[i] == '-update_rate':
        if len(sys.argv) > i+1:
          self.update_rate = float(sys.argv[i+1])
      if sys.argv[i] == '-timeout':
        if len(sys.argv) > i+1:
          self.timeout = float(sys.argv[i+1])
      if sys.argv[i] == '-x':
        if len(sys.argv) > i+1:
          self.target_l[0] = float(sys.argv[i+1])
      if sys.argv[i] == '-y':
        if len(sys.argv) > i+1:
          self.target_l[1] = float(sys.argv[i+1])
      if sys.argv[i] == '-z':
        if len(sys.argv) > i+1:
          self.target_l[2] = float(sys.argv[i+1])
      if sys.argv[i] == '-R':
        if len(sys.argv) > i+1:
          self.target_e[0] = float(sys.argv[i+1])
      if sys.argv[i] == '-P':
        if len(sys.argv) > i+1:
          self.target_e[1] = float(sys.argv[i+1])
      if sys.argv[i] == '-Y':
        if len(sys.argv) > i+1:
          self.target_e[2] = float(sys.argv[i+1])
      if sys.argv[i] == '-t':
        if len(sys.argv) > i+1:
          self.topic_name = sys.argv[i+1]

    # setup rospy
    self.pub_set_wrench_topic = rospy.Publisher(self.topic_name, Wrench)

    # compoose goal message
    w = Wrench(Vector3(self.target_l[0],self.target_l[1],self.target_l[2]),Vector3(self.target_e[0],self.target_e[1],self.target_e[2]))

    # publish topic if specified
    timeout_t = time.time() + self.timeout
    while not rospy.is_shutdown() and time.time() < timeout_t:
      # publish target wrench
      self.pub_set_wrench_topic.publish(w)

      if self.update_rate > 0:
        time.sleep(1.0/self.update_rate)
      else:
        time.sleep(0.001)

def print_usage(exit_code = 0):
    print('''Commands:
    -update_rate <Hz> - update rate, default to 10 Hz
    -timeout <seconds> - test timeout in seconds. default to 1 seconds
    -x <x in meters>
    -y <y in meters>
    -z <z in meters>
    -R <roll in radians>
    -P <pitch in radians>
    -Y <yaw in radians>
    -t set wrench topic name
''')


if __name__ == '__main__':
    #print usage if not arguments
    if len(sys.argv) == 1:
      print_usage()
    else:
      sic = SimIfaceControl()
      sic.setWrench()



