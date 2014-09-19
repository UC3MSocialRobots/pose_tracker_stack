#!/usr/bin/env python
PKG = 'pose_labeler'
NNAME = 'test_pose_labeler_bad_init'
import roslib; roslib.load_manifest(PKG)
# import rospy
import unittest
import pose_labeler_node as pln

class PoseLabelerBadInit(unittest.TestCase):
    """Tests for PoseLabeler bad initialization.
        ASSUMES that the node will NOT receive any parameters.
    """
    def __init__(self, *args):
        super(PoseLabelerBadInit, self).__init__(*args)
            
    def setUp(self):
        pass

    def test_pose_label_bad_init(self):
        self.assertRaises(pln.PoseLabeler)
        

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_pose_labeler_bad_init', PoseLabelerBadInit)
    # rostest.rosrun(PKG, 'pose_labeler_init', PoseLabelerInitTestCase)
    
    # import rosunit
    # rosunit.unitrun(PKG, 'PoseLabeler_oneequalsone', PoseLabelerTestCase)
