#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-05
'''
from __future__ import division
PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)

import rospy
import unittest
import numpy as np
import pandas as pd

from joint_velocities_publisher import JointVelocitiesPublisher


class TestJointVelocitiesPublisher(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestJointVelocitiesPublisher, self).__init__(*args)

    def setUp(self):
        ''' :todo: load node
            :todo: create dataframe that later will be used as instances
        '''
        self.df = pd.DataFrame(np.linspace(1,20,20).reshape(4,5),
                               columns=list('ABCDE'))
        self.node = JointVelocitiesPublisher()
        # node.run()

    def tearDown(self):
        pass
        # import rosnode
        # rosnode.kill_nodes(['/instance_averager_node'])

    def test_calc_velocities(self):
        ''' give some datasets and check if velocities are correctly calculated'''
        # Pre-compute results
        velocities = np.array([16-1, 17-2, 18-3, 19-4, 20-5]) / 4
        comparison = self.node.calc_velocities(self.df) == velocities
        self.assertTrue(all(comparison))
    
    def test_calc_velocities_returns_0_if_passed_a_single_row_dataframe(self):
        df = pd.DataFrame()
        df = df.append(pd.Series(range(5)), ignore_index=True)
        comparison = self.node.calc_velocities(df) == np.zeros(5)
        self.assertTrue(all(comparison))
        
    def test_calc_velocities_raises_ValueError_if_dataframe_is_empty(self):
        with self.assertRaises(ValueError):
            self.node.calc_velocities(pd.DataFrame())


    def test_publishes_velocities_when_receives_instances(self):
        '''send df as pose_instances and check if returned velocities are ok
           do it with a generator, so i can control the flow of messages calling
           next()'''
        pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_joint_velocities_publisher',
                   TestJointVelocitiesPublisher)