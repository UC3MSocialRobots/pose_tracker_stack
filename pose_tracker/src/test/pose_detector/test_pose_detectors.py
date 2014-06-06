#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-06
'''
from __future__ import division
PKG = 'pose_tracker'
import roslib
roslib.load_manifest(PKG)
# import rospy
# from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

# import itertools as it
import unittest
import numpy as np
import pandas as pd

from pose_detector.pose_detector_node import (is_dataset_full, is_still,
                                              is_moving, DatasetNotFullError)


class TestIsDatasetFull(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestIsDatasetFull, self).__init__(*args)
        # name = 'test_pose_detector'
        # rospy.init_node(name)

    def setUp(self):
        # datasets with 1,2,3,4 rows and 5 columns
        self.datasets = (pd.DataFrame(np.linspace(1, 5, 5).reshape(1, 5)),
                         pd.DataFrame(np.linspace(1, 10, 10).reshape(2, 5)),
                         pd.DataFrame(np.linspace(1, 15, 15).reshape(3, 5)),
                         pd.DataFrame(np.linspace(1, 20, 20).reshape(4, 5)))

    def tearDown(self):
        pass

    def test_is_dataset_full_raises_if_not_full(self):
        for i, expected_length in enumerate([2, 3, 4, 5]):
            with self.assertRaises(DatasetNotFullError):
                # dataset_length = len(self.datasets[i])
                is_dataset_full(expected_length, self.datasets[i])

    def test_is_dataset_full_raises_if_len_dataset_greater_than_length(self):
        for i, expected_length in enumerate([0, 1, 1, 1]):
            # dataset_length = len(self.datasets[i])
            with self.assertRaises(DatasetNotFullError):
                is_dataset_full(expected_length, self.datasets[i])

    def test_is_dataset_full_does_not_raise(self):
        for i, expected_length in enumerate([1, 2, 3, 4]):
            # dataset_length = len(self.datasets[i])
            try:
                is_dataset_full(expected_length, self.datasets[i])
            except DatasetNotFullError as dnf:
                self.fail(dnf)


class TestIsStill(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestIsStill, self).__init__(*args)
        # name = 'test_pose_detector'
        # rospy.init_node(name)

    def setUp(self):
        self.zeros = pd.DataFrame(np.zeros(15).reshape(3, 5)),
        self.point_ones = pd.DataFrame(np.ones(15).reshape(3, 5) * 0.1)
        self.fives = pd.DataFrame(np.ones(15).reshape(3, 5) * 5)
        self.tens = pd.DataFrame(np.ones(15).reshape(3, 5) * 10)
        self.datasets = (self.zeros, self.point_ones, self.fives, self.tens)

    def tearDown(self):
        pass

    def returns_True_if_all_data_is_below_threshold(self):
        for dataset in self.datasets:
            self.assertTrue(is_still(11, dataset))

        self.minustens = self.tens * (-1)
        self.assertTrue(is_still(3), self.minustens)

    def returns_false_if_some_data_is_equal_to_threshold(self):
        for dataset in self.datasets:
            self.assertFalse(is_still(len(dataset), dataset))

    def returns_False_if_some_data_is_above_threshold(self):
        self.zeros.iloc[1, 1] = 13
        for dataset in [self.zeros, self.tens]:
            self.assertFalse(is_still(6, dataset))


class TestIsMoving(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestIsMoving, self).__init__(*args)
        # name = 'test_pose_detector'
        # rospy.init_node(name)

    def setUp(self):
        self.zeros = pd.DataFrame(np.zeros(15).reshape(3, 5)),
        self.point_ones = pd.DataFrame(np.ones(15).reshape(3, 5) * 0.1)
        self.fives = pd.DataFrame(np.ones(15).reshape(3, 5) * 5)
        self.tens = pd.DataFrame(np.ones(15).reshape(3, 5) * 10)
        self.datasets = (self.zeros, self.point_ones, self.fives, self.tens)

    def tearDown(self):
        pass

    def returns_False_if_some_data_is_below_threshold(self):
        self.tens.iloc[1, 1] = -13
        self.zeros.iloc[1, 1] = 25
        for dataset in [self.zeros, self.fives, self.tens]:
            self.assertFalse(is_moving(6, dataset))

    def returns_false_if_some_data_is_equal_to_threshold(self):
        for dataset in self.datasets:
            self.assertFalse(is_moving(len(dataset), dataset))

    def returns_True_if_all_data_is_above_threshold(self):
        for dataset in self.datasets[1:]:
            self.assertTrue(is_moving(0, dataset))


class TestIsStillAndIsMovingAreMutuallyExclusive(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestIsStillAndIsMovingAreMutuallyExclusive, self).__init__(*args)
        # name = 'test_pose_detector'
        # rospy.init_node(name)

    def setUp(self):
        self.zeros = pd.DataFrame(np.zeros(15).reshape(3, 5)),
        self.point_ones = pd.DataFrame(np.ones(15).reshape(3, 5) * 0.1)
        self.fives = pd.DataFrame(np.ones(15).reshape(3, 5) * 5)
        self.tens = pd.DataFrame(np.ones(15).reshape(3, 5) * 10)
        self.datasets = (self.zeros, self.point_ones, self.fives, self.tens)

    def tearDown(self):
        pass

    def test_is_still_and_is_moving_give_contrary_results(self):
        for dataset in self.datasets:
            isstill = is_still(3, dataset)
            ismoving = is_moving(3, dataset)
            self.assertNotEqual(isstill, ismoving,
                                "{} == {}\n"
                                "is_still and is_moving gave same results "
                                "for threshold {} and dataset:\n{}"
                                .format(isstill, ismoving, 3, dataset))

    def test_is_still_and_is_moving_produce_same_results(self):
        self.fail('TODO')

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_is_dataset_full', TestIsDatasetFull)
    rosunit.unitrun(PKG, 'test_is_still', TestIsStill)
    rosunit.unitrun(PKG, 'test_is_moving', TestIsMoving)
    rosunit.unitrun(PKG, 'test_is_moving',
                    TestIsStillAndIsMovingAreMutuallyExclusive)
