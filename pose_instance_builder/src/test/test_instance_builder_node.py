#!/usr/bin/python
PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)
import rospy

import unittest

from instance_builder_node import InstanceBuilderNode
from instance_builder import (PiTrackerIBuilder, KinectIBuilder)

from param_utils import get_parameters

_BUILDERS = {'instance_builder.KinectIBuilder': KinectIBuilder, 
             'instance_builder.PiTrackerIBuilder': PiTrackerIBuilder}

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
        
    def test_node_loads_correct_builder(self):
        node = InstanceBuilderNode()
        self.p = get_parameters('builder_type').next()
        self.assertEqual(self.p.value, node.builder_type)
        self.assertTrue(isinstance(node.builder, _BUILDERS[self.p.value]))

    # def test_node_inserts_label_to_instances(self):
    #     self.fail("ToDo: 1) Write test. 2) Implement functionality")

    # def test_cb_returns_appropriate_instance(self):
    #     pass
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_InstanceBuilderNode', TestInstanceBuilderNode)

