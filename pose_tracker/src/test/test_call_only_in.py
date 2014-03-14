#!/usr/bin/env python
PKG = 'pose_tracker'
NNAME = 'test_PoseDatasetBuilder_call_only_in'
import roslib; roslib.load_manifest(PKG)
# import rospy
import unittest

import pose_tracker.pose_dataset_builder_node as pdb

class TestCallOnlyIn(unittest.TestCase):
    """Tests"""
    class MyClass():
        ''' Helper class '''
        def __init__(self):
            self.states = pdb.ALL_STATES
            self.curr_state = pdb.STATE_INIT
      
        def change_state(self, state):
            if state in self.states:
                self.curr_state = state
            
        @pdb.call_only_in(pdb.STATE_INIT)
        def state1(self):
            # print "Doing state 1"
            return 1
        
        @pdb.call_only_in([pdb.STATE_IDLE, pdb.STATE_PROCESSING])
        def state2(self):
            # print "Doing state 2"
            return 2
        
        @pdb.call_only_in(pdb.STATE_PROCESSING)
        def state3(self):
            # print "Doing state 3"
            return 3

    def __init__(self, *args):
        super(TestCallOnlyIn, self).__init__(*args)
             
    def setUp(self):
        self.mc = TestCallOnlyIn.MyClass()
        
    def tearDown(self):
        pass

    def test_call_only_in_works_if_states_are_correct(self):
        self.mc.curr_state = pdb.STATE_INIT
        self.assertEqual(self.mc.state1(), 1, 
            "'state1()' should have been called in s1")
        self.assertEqual(self.mc.state2(), None,
            "'state2()' should NOT have been called in s1")
        self.assertEqual(self.mc.state3(), None, 
            "'state3()' should NOT have been called in s1")

        self.mc.curr_state = pdb.STATE_PROCESSING
        self.assertEqual(self.mc.state1(), None,
            "'state1()' should NOT have been called in s3")
        self.assertEqual(self.mc.state2(), 2,
            "'state2()' should have been called in s3")
        self.assertEqual(self.mc.state3(), 3,
            "'state3()' should have been called in s3")



if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_CallOnlyIn', TestCallOnlyIn)

