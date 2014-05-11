#!/usr/bin/python
PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)


import unittest

from instance_builder_node import InstanceBuilderNode
from instance_builder import (PiTrackerIBuilder, KinectIBuilder)
# from pi_tracker.msg import Skeleton
# from geometry_msgs.msg import (Vector3, Quaternion)
# from pose_instance_builder.msg import PoseInstance


class TestInstanceBuilderNode(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestInstanceBuilderNode, self).__init__(*args)
        
    def setUp(self):
        pass

    def tearDown(self):
        pass
    
    def test_instantiate_node(self):
        try:
            InstanceBuilderNode()
        except:
            self.fail()
        
        
    def cb_returns_appropriate_instance():
        pass
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_InstanceBuilderNode', TestInstanceBuilderNode)

