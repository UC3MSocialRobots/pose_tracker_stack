#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-05
'''
from __future__ import division
PKG = 'pose_tracker'
import roslib
roslib.load_manifest(PKG)
# import rospy
# import itertools as it
import numpy as np
import pandas as pd
from pose_detector.joint_velocities_publisher import calc_velocities
import unittest
from numpy.testing import assert_array_almost_equal as assert_arrAlmostEQ


class TestCalcJointVelocities(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestCalcJointVelocities, self).__init__(*args)

    def setUp(self):
        ''' :todo: load node
            :todo: create dataframe that later will be used as instances
        '''
        self.df = pd.DataFrame(np.linspace(1, 20, 20).reshape(4, 5),
                               columns=list('ABCDE'))
        # Pre-compute velocities
        self.df_velocities = {1: np.array([1 - 1, 2 - 2, 3 - 3, 4 - 4, 5 - 5]),
                              2: np.array([6 - 1, 7 - 2, 8 - 3, 9 - 4, 10 - 5]) / 2,
                              3: np.array([11 - 1, 12 - 2, 13 - 3, 14 - 4, 15 - 5]) / 3,
                              4: np.array([16 - 1, 17 - 2, 18 - 3, 19 - 4, 20 - 5]) / 4}

    def tearDown(self):
        pass

    def test_calc_velocities(self):
        velocities = self.df_velocities[len(self.df)]
        expected_velocities = calc_velocities(self.df)
        assert_arrAlmostEQ(expected_velocities, velocities)

    def test_calc_velocities_returns_0_if_passed_a_single_row_dataframe(self):
        df = pd.DataFrame()
        df = df.append(pd.Series(range(5)), ignore_index=True)
        expected_velocities = calc_velocities(df)
        assert_arrAlmostEQ(expected_velocities, np.zeros(5))

    def test_calc_velocities_raises_ValueError_if_dataframe_is_empty(self):
        with self.assertRaises(ValueError):
            calc_velocities(pd.DataFrame())


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_calc_joint_velocities', TestCalcJointVelocities)
