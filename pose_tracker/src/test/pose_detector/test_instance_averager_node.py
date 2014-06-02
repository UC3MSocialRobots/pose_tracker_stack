'''
@author: Victor Gonzalez Pacheco
@date: 2014-05
'''
PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)

import rospy
import unittest
import pandas as pd

from instance_averager_node import InstanceAveragerNode


class TestInstanceAveragerNode(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestInstanceAveragerNode, self).__init__(*args)

    def setUp(self):
        ian = InstanceAveragerNode()
        ian.run()
        
    def tearDown(self):
        pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_instance_averager_node', TestInstanceAveragerNode)