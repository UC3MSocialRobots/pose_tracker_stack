PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)


import unittest

from instance_builder_node import (load_instance_builder, load_params, 
                                   InstanceBuilderNode)
from instance_builder import (parse_label, PiTrackerIBuilder, KinectIBuilder)
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
    
    def test_load_instance_builder(self):
        for builder in ('PiTrackerIBuilder', 'KinectIBuilder'):
            instance = load_instance_builder(builder)
            self.assertTrue(isinstance(instance, builder))

    def test_load_params(self):
        try:
            param = load_params()
            self.assertEqual(param, 'PiTrackerIBuilder')
        except:
            self.fail("Failed loading parameters")

    def test_instantiate_node(self):
        pass

    def cb_returns_appropriate_instance():
        pass
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_InstanceBuilderNode', TestInstanceBuilderNode)

