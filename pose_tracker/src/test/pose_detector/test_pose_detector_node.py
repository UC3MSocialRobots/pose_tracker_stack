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
from pose_detector import (DatasetNotFullError, PoseDetectorNode,
                           is_dataset_full, is_still, is_moving)


class TestPoseDetectorNode(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestPoseDetectorNode, self).__init__(*args)
        name = 'test_pose_detector'
        # rospy.init_node(name)
        self.node = PoseDetectorNode()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def __fill_velocities_dataframe(self):
        data = np.linspace(1, 10, 10).reshape(2, 5)
        self.node.velocities = pd.DataFrame(data)

    def test_change_state_selects_proper_detector_in_each_change(self):
        ''' Tests if detector changes properly.'''
        for expected_detector in [is_moving, is_still, is_moving, is_still]:
            self.__fill_velocities_dataframe()
            self.node.change_state()
            self.assertEqual(expected_detector, self.node.current_detector)

    def test_change_state_flushes_velocities_dataframe(self):
        for expected_detector in [is_moving, is_still, is_moving, is_still]:
            self.__fill_velocities_dataframe()
            self.node.change_state()
            self.assertEqual(0, len(self.node.velocities))

    def test_velocities_cb(self):
        self.fail("TODO")

    def test_velocities_cb_publishes_an_instance_when_the_user_is_still(self):
        pass

    def test_velocities_cb_publishes_velos_when_the_user_starts_moving(self):
        pass


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_pose_detector_node', TestPoseDetectorNode)
