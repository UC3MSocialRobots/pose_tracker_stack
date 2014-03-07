#!/usr/bin/env python
PKG = 'pose_labeler'
NNAME = 'test_pose_labeler_good_init'
import roslib; roslib.load_manifest(PKG)
import rospy
import unittest
import thread
import pose_labeler_node as pln

class PoseLabelerInitsWell(unittest.TestCase):
    """Tests for PoseLabeler"""
    def __init__(self, *args):
        super(PoseLabelerInitsWell, self).__init__(*args)
            
    def setUp(self):
        pass

    def test_pose_label_good_init(self):
        args = ()  # no args needed
        thread.start_new_thread(self.launch_pose_labeler_node, args)
        rospy.sleep(0.5)
        rospy.signal_shutdown("node " + rospy.get_name() \
                + " Exited cleanly")

    def launch_pose_labeler_node(self):
        try:
            node = pln.PoseLabeler()
        except:
            self.fail("PoseLabeler died unexpectedly!")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_pose_labeler_good_init', PoseLabelerInitsWell)
    # rostest.rosrun(PKG, 'pose_labeler_init', PoseLabelerInitTestCase)
    
    # import rosunit
    # rosunit.unitrun(PKG, 'PoseLabeler_oneequalsone', PoseLabelerTestCase)
