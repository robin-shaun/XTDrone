#!/usr/bin/env python

"""
Test plugin gazebo_ros_block_laser using gazebo_ros_block_laser_clipping.world.
"""

import math
import rospy
from sensor_msgs.msg import PointCloud

import unittest

class TestBlockLaserClippingPlugin(unittest.TestCase):
    MIN_RANGE = 0.4
    MAX_RANGE = 1.0

    def _ranges(self):
        msg = rospy.wait_for_message('test_block_laser', PointCloud)
        for point in msg.points:
            yield math.sqrt(point.x**2 + point.y**2 + point.z**2)

    def test_points_at_all_depths(self):
        # Make sure there are points at all depths
        BIN_SIZE = 0.01
        num_bins = int((self.MAX_RANGE - self.MIN_RANGE) / BIN_SIZE)
        bins = [b * BIN_SIZE + self.MIN_RANGE for b in range(num_bins)]
        bin_counts = dict(zip(bins, [0] * len(bins)))

        for r in self._ranges():
            closest_bin = None
            closest_dist = float('inf')
            for b in bins:
                dist = abs(r - b)
                if abs(r - b) < closest_dist:
                    closest_dist = dist
                    closest_bin = b
            bin_counts[closest_bin] += 1

        self.assertTrue(0 not in bin_counts.values(), msg=repr(bin_counts))

    def test_between_min_max(self):
        ALLOWED_ERROR = 0.00001
        for r in self._ranges():
            self.assertGreater(r, self.MIN_RANGE - ALLOWED_ERROR)
            self.assertLess(r, self.MAX_RANGE + ALLOWED_ERROR)


if __name__ == '__main__':
    import rostest
    PKG_NAME = 'gazebo_plugins'
    TEST_NAME = PKG_NAME + 'block_laser_clipping'
    rospy.init_node(TEST_NAME)
    rostest.rosrun(PKG_NAME, TEST_NAME, TestBlockLaserClippingPlugin)
