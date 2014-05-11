#!/usr/bin/python
PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)


import unittest
from mock import patch

from instance_builder_node import (load_instance_builder, load_params,
                                   InstanceBuilderNode)
from instance_builder import (PiTrackerIBuilder, KinectIBuilder)
from pi_tracker.msg import Skeleton
from geometry_msgs.msg import (Vector3, Quaternion)
from pose_instance_builder.msg import PoseInstance


class TestInstanceBuilderNode(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestInstanceBuilderNode, self).__init__(*args)
        
    def setUp(self):
        pass

    def tearDown(self):
        pass
    
    # def test_load_instance_builder(self):
    #     builders = {'PiTrackerIBuilder': PiTrackerIBuilder, 
    #                 'KinectIBuilder': KinectIBuilder}
    #     for bname, builder in builders.items():
    #         instance = load_instance_builder(bname)
    #         instance()
    #         self.assertTrue(isinstance(instance, builder))

    def test_load_params(self):
        # node = InstanceBuilderNode()
        # # builder_type, topic = '', ''
        # # try:
        # #     builder_type, topic = node.load_params()
        # # except:
        # #     self.fail("Failed loading parameters")
        # # self.assertEqual(builder_type, PiTrackerIBuilder)


        # params = ['~builder_type, skeleton_topic']
        # try:
        #     builder_type, topic = load_params(params)
        #     self.assertEqual(builder_type, PiTrackerIBuilder)
        # except Exception, e:
        #     self.fail("Failed loading parameters. Reason: {}"
        #               .format(e.message))
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

