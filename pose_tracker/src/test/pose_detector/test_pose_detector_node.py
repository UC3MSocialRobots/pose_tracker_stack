#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-05
'''
from __future__ import division
PKG = 'pose_tracker'
import roslib
roslib.load_manifest(PKG)
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

# import itertools as it
import unittest
import numpy as np
import pandas as pd

from pose_msgs.msg import (PoseInstance, JointVelocities)
from pose_detector.pose_detector_node import (DatasetNotFullError,
                                              PoseDetectorNode, Detector,
                                              is_dataset_full,
                                              is_still, is_moving)


class TestPoseDetectorNode(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestPoseDetectorNode, self).__init__(*args)
        name = 'test_pose_detector'
        rospy.init_node(name)
        # self.node = PoseDetectorNode()

    def setUp(self):
        dstill = self.node._still_detector
        dmoving = self.node._moving_detector
        self.test_detectors = [dmoving, dstill] * 3

    def tearDown(self):
        pass

    @unittest.skip('TODO')
    def test_velocities_cb(self):
        self.fail("TODO")

    @unittest.skip('TODO')
    def test_velocities_cb_publishes_an_instance_when_the_user_is_still(self):
        self.fail('TODO')

    @unittest.skip('TODO')
    def test_velocities_cb_publishes_velos_when_the_user_starts_moving(self):
        self.fail('TODO')

    @unittest.skip('TODO')
    def test_Node_switches_detector_when_user_starts_stops_moving(self):
        self.fail('TODO')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_pose_detector_node', TestPoseDetectorNode)
