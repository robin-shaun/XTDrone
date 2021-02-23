#!/usr/bin/python
# Copyright 2018 Joanthan Wheare
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import random
import rospy

from geometry_msgs.msg import Vector3
from usv_msgs.msg import RangeBearing
from visualization_msgs.msg import Marker

"""@package set_pinger_position.py
Automatically generate a position for a simulated USV pinger.  This
position is published both as a Vector3 to the plugin, and as a Marker that
can be visualised.  This visualisation can be used as a ground truth to
compare with the estimated pinger position.
Possible positions are stored as ROS parameters.  The node assumes that
positions will be three element vectors each stored in the nodes parameter
space in the format position_n starting with position_1.  If no positions
are available, the node will default to the origin.
"""


class PingerPosition:
    """Class used to store the parameters and variables for the script.
    """
    def __init__(self):
        """Initialise and run the class."""
        rospy.init_node("set_pinger_position")

        # Load the positions of the pingers.
        self.pingerPositions = list()
        i = 1
        while rospy.has_param('~position_' + str(i)):
            self.pingerPositions.append(rospy.get_param('~position_' + str(i)))
            i = i + 1

        # If there are no matching positions, initialise to the origin.
        if i == 1:
            self.pingerPositions.append([0, 0, 0])
        self.pingerPub = rospy.Publisher("/wamv/sensors/pingers/pinger/set_pinger_position",
                                         Vector3, queue_size=10, latch=True)
        self.markerPub = rospy.Publisher("/wamv/sensors/pingers/pinger/marker/ground_truth", Marker,
                                         queue_size=10, latch=True)

        while not rospy.is_shutdown():

            # Choose a position for the pinger.
            position = random.choice(self.pingerPositions)

            # Create a vector message.
            msg = Vector3()
            msg.x = position[0]
            msg.y = position[1]
            msg.z = position[2]
            self.pingerPub.publish(msg)

            msg = Marker()
            msg.header.frame_id = "/map"
            msg.header.stamp = rospy.get_rostime()
            msg.ns = ""
            msg.id = 0
            msg.type = Marker.SPHERE
            msg.action = Marker.ADD

            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            msg.scale.x = 1.0
            msg.scale.y = 1.0
            msg.scale.z = 1.0

            msg.color.r = 1.0
            msg.color.g = 0.0
            msg.color.b = 0.0
            msg.color.a = 1.0

            self.markerPub.publish(msg)

            # Change position every 10 seconds.
            rospy.sleep(10.)


if __name__ == '__main__':
    pinger = PingerPosition()
