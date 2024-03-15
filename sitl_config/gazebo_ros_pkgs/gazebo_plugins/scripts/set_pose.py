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

## Gazebo send position topic or calls set pose service for ros_sim_iface consumption

PKG = 'gazebo_plugins'
NAME = 'set_pose'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest

from gazebo_plugins.srv import SetPose

from std_msgs.msg import String
from geometry_msgs.msg import Pose,Quaternion,Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3
from nav_msgs.msg import Odometry
import tf.transformations as tft
from numpy import float64

COV = [float64(0),float64(0),float64(0),float64(0),float64(0),float64(0), \
       float64(0),float64(0),float64(0),float64(0),float64(0),float64(0), \
       float64(0),float64(0),float64(0),float64(0),float64(0),float64(0), \
       float64(0),float64(0),float64(0),float64(0),float64(0),float64(0), \
       float64(0),float64(0),float64(0),float64(0),float64(0),float64(0), \
       float64(0),float64(0),float64(0),float64(0),float64(0),float64(0)  ]


def normalize_angle_positive(angle):
    return math.fmod(math.fmod(angle, 2*math.pi) + 2*math.pi, 2*math.pi)

def normalize_angle(angle):
    anorm = normalize_angle_positive(angle)
    if anorm > math.pi:
      anorm -= 2*math.pi
    return anorm

def shortest_angular_distance(angle_from, angle_to):
    angle_diff = normalize_angle_positive(angle_to) - normalize_angle_positive(angle_from)
    if angle_diff > math.pi:
      angle_diff = -(2*math.pi - angle_diff)
    return normalize_angle(angle_diff)

class SimIfaceControl():
  def __init__(self):
    self.update_rate=10
    self.timeout=1
    self.target_p = [0,0,0] # position
    self.target_q = [0,0,0] # quaternion
    self.target_e = [0,0,0] # euler pose
    self.wait_topic_initialized = False
    self.frame_id = "world"
    self.service_name = "set_pose_service"
    self.topic_name = "set_pose_topic"
    self.use_topic = False;
    self.use_service = False;
    self.wait_topic_name = "clock"
    self.wait_for_topic = False;

    rospy.init_node(NAME, anonymous=True)

  def setPoseService(self,pose_msg):
    print('waiting for service to set pose')
    rospy.wait_for_service(self.service_name);
    try:
      set_pose = rospy.ServiceProxy(self.service_name, SetPose)
      resp1 = set_pose(pose_msg)
      return resp1.success
    except rospy.ServiceException as e:
      print("service call failed: %s" % e)

  def waitTopicInput(self,p3d):
    #self.p3d_p = [p3d.pose.pose.position.x, p3d.pose.pose.position.y, p3d.pose.pose.position.z]
    #self.p3d_q = [p3d.pose.pose.orientation.x, p3d.pose.pose.orientation.y, p3d.pose.pose.orientation.z, p3d.pose.pose.orientation.w]
    #self.p3d_e = tft.euler_from_quaternion(self.p3d_q)
    self.wait_topic_initialized = True

  def setPose(self):
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
          self.target_p[0] = float(sys.argv[i+1])
      if sys.argv[i] == '-y':
        if len(sys.argv) > i+1:
          self.target_p[1] = float(sys.argv[i+1])
      if sys.argv[i] == '-z':
        if len(sys.argv) > i+1:
          self.target_p[2] = float(sys.argv[i+1])
      if sys.argv[i] == '-R':
        if len(sys.argv) > i+1:
          self.target_e[0] = float(sys.argv[i+1])
      if sys.argv[i] == '-P':
        if len(sys.argv) > i+1:
          self.target_e[1] = float(sys.argv[i+1])
      if sys.argv[i] == '-Y':
        if len(sys.argv) > i+1:
          self.target_e[2] = float(sys.argv[i+1])
      if sys.argv[i] == '-f':
        if len(sys.argv) > i+1:
          self.frame_id = sys.argv[i+1]
      if sys.argv[i] == '-s':
        if len(sys.argv) > i+1:
          self.service_name = sys.argv[i+1]
          self.use_service = True;
      if sys.argv[i] == '-t':
        if len(sys.argv) > i+1:
          self.topic_name = sys.argv[i+1]
          self.use_topic = True;
      if sys.argv[i] == '-p':
        if len(sys.argv) > i+1:
          self.wait_topic_name = sys.argv[i+1]
          self.wait_for_topic = True;

    # setup rospy
    self.pub_set_pose_topic = rospy.Publisher(self.topic_name, Odometry)
    rospy.Subscriber(self.wait_topic_name, rospy.AnyMsg, self.waitTopicInput)

    # wait for topic if user requests
    if self.wait_for_topic:
      while not self.wait_topic_initialized:
        time.sleep(0.1)

    # compoose goal message
    h = rospy.Header()
    h.stamp = rospy.get_rostime()
    h.frame_id = self.frame_id
    p = Point(self.target_p[0],self.target_p[1],self.target_p[2])
    tmpq = tft.quaternion_from_euler(self.target_e[0],self.target_e[1],self.target_e[2])
    q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
    pose = Pose(p,q)
    pwc = PoseWithCovariance(pose,COV)
    twc = TwistWithCovariance(Twist(Vector3(),Vector3()),COV)
    child_frame_id = "" # what should this be?
    target_pose = Odometry(h,child_frame_id,pwc,twc)

    if self.use_service:
      success = self.setPoseService(target_pose)

    # publish topic if specified
    if self.use_topic:
      timeout_t = time.time() + self.timeout
      while not rospy.is_shutdown() and time.time() < timeout_t:
        # publish target pose
        self.pub_set_pose_topic.publish(target_pose)

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
    -f target frame_id
    -s set pose service name
    -t set pose topic name
    -p wait for this ros topic to be published first
''')

if __name__ == '__main__':
    #print usage if not arguments
    if len(sys.argv) == 1:
      print_usage()
    else:
      sic = SimIfaceControl()
      sic.setPose()



