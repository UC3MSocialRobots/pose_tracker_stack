#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-05
'''
from __future__ import division
PKG = 'pose_tracker'
import roslib
roslib.load_manifest(PKG)

import unittest
import numpy as np
import pandas as pd


from pose_detector.pose_detector_node import PoseDetectorNode


class TestPoseDetectorChangesProperly(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestPoseDetectorChangesProperly, self).__init__(*args)
        self.node = PoseDetectorNode()

    def setUp(self):
        dstill = self.node._still_detector
        dmoving = self.node._moving_detector
        self.test_detectors = [dmoving, dstill] * 3

    def tearDown(self):
        pass

    def __fill_velocities_dataframe(self):
        data = np.linspace(1, 10, 10).reshape(2, 5)
        self.node.velocities = pd.DataFrame(data)

    def __change_detector(self):
        self.__fill_velocities_dataframe()
        self.node.change_detector(self.node.detectors)

    def test_change_detector_selects_proper_detector_in_each_change(self):
        ''' Tests if detector changes properly.'''
        for expected_detector in self.test_detectors:
            self.__change_detector()
            self.assertEqual(expected_detector, self.node.current_detector)

    def test_change_detector_flushes_velocities_dataframe_after_changes(self):
        for expected_detector in self.test_detectors:
            self.__change_detector()
            self.assertEqual(0, len(self.node.velocities))


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_pose_detector_node_changes_detectors_properly',
                   TestPoseDetectorChangesProperly)
                   # coverage_packages=['pose_detector.pose_detector_node.py'])
