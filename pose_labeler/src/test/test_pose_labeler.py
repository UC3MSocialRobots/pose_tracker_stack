#!/usr/bin/env python
PKG = 'pose_labeler'
NNAME = 'test_pose_labeler'
import roslib; roslib.load_manifest(PKG)
import rospy
import unittest
import asr_msgs.msg as asrmsg
from std_msgs.msg import String
import copy

# import pose_labeler_node as pln

class PoseLabelerTestCase(unittest.TestCase):
    """Tests for PoseLabeler"""
    def __init__(self, *args):
        super(PoseLabelerTestCase, self).__init__(*args)
            
    def setUp(self):
        # self.pl = pln.PoseLabeler()
        rospy.init_node(NNAME)
        rospy.loginfo("Initializing " + rospy.get_name() + " node...")

        self.asrpub = rospy.Publisher("recognition_results", 
                                      asrmsg.recog_results)
        self.label_sub = rospy.Subscriber('pose_label',
                                         String,
                                         self.label_callback)
        self.comand_sub = rospy.Subscriber('pose_command',
                                         String,
                                         self.command_callback)
        self.pose_labels = rospy.get_param('pose_labels')
        self.commands = rospy.get_param('pose_commands')

        # setting parameters for pose_labeler node:
        rospy.set_param('pose_labeler/min_confidence', 0.5)
        self.label = ''
        self.command = ''
        # self.min_confidence = rospy.get_param('~min_confidence', 0.5)

        self.correct_pose_messages = self.fill_msgs(self.pose_labels)
        self.correct_commands = self.fill_msgs(self.commands)
        
        
    # def tearDown(self):
    #     pass

    ####### CALLBACKS #######
    def label_callback(self, label):
        rospy.loginfo("Received label: " + label.data)
        self.label = label.data

    def command_callback(self, command):
        rospy.loginfo("Received command: " + command.data)
        self.command = command.data
    
    ####### HELPER FUNCTIONS #######
    def fill_msgs(self, msg_list, confidence=0.5):
        ''' Helper function that converts a list of strings 
            into a list of asr msgs'''
        mlist = [] # the list of asrmsgs/recognition_results messages
        for message in msg_list:
            msg = asrmsg.recog_results()
            message_chunks = message.split('_')
            for mc in message_chunks:
                attvalue = asrmsg.attribute_value()
                attvalue.value = mc
                attvalue.confidence = confidence
                msg.semantics.append(attvalue)
            mlist.append(msg)
        return mlist

    ####### TESTS #######
    def test_confidence_threshold(self):
        r = rospy.Rate(5) # Hz
        rospy.set_param('pose_labeler/min_confidence', 0.5)
        rospy.sleep(1)
        lower_msg = copy.copy(self.correct_pose_messages[0])
        equal_msg = copy.copy(self.correct_pose_messages[1])
        greater_msg = copy.copy(self.correct_pose_messages[2])
        
        for i,s in enumerate(lower_msg.semantics):
            lower_msg.semantics[i].confidence = 0.1
            equal_msg.semantics[i].confidence = 0.5
            greater_msg.semantics[i].confidence = 0.7

        self.asrpub.publish(lower_msg)
        # rospy.sleep(0.5)
        r.sleep()
        self.assertEqual(self.label, '', "Should not have received a label")
        self.assertEqual(self.command, '', "Should not have received a command")
        self.label = ''

        self.asrpub.publish(equal_msg)
        # rospy.sleep(1)
        r.sleep()
        self.assertNotEqual(self.label, '', "Should have received a label")
        self.assertEqual(self.command, '', "Should not have received a command")
        self.label = ''

        self.asrpub.publish(greater_msg)
        # rospy.sleep(1)
        r.sleep()
        self.assertNotEqual(self.label, '', "Should have received a label")
        self.assertEqual(self.command, '', "Should not have received a command")
    
    def test_send_correct_labels(self):
        r = rospy.Rate(10) # in Hz
        for i,m in enumerate(self.correct_pose_messages):
            self.asrpub.publish(m)
            r.sleep()
            self.assertEqual(self.label, self.pose_labels[i],
                             "Received label (%s) differs from expected (%s)"
                             % (self.label, self.pose_labels[i]))

    def test_send_correct_commands(self):
        r = rospy.Rate(10) # in Hz
        for i,c in enumerate(self.correct_commands):
            self.asrpub.publish(c)
            r.sleep()
            self.assertEqual(self.command, self.commands[i],
                             "Received command (%s) differs from expected (%s)"
                             % (self.command, self.commands[i]))

    def test_unexpected_asr_msg(self):
        '''tests labels that is not in label_dict neither "UNKNOWN" label '''
        attvalue = asrmsg.attribute_value()
        attvalue.atribute = "unkkknownn pppose"
        attvalue.value = "not a ppppose"
        attvalue.confidence = 0.9

        msg = asrmsg.recog_results()
        msg.semantics.append(attvalue)
        self.asrpub.publish(msg)
        rospy.sleep(1)
        self.assertEquals(self.label, 'UNKNOWN', "label != UNKNOWN")
        self.assertEquals(self.command, 'UNKNOWN', "command != UNKNOWN")

    def test_empty_asr_msg(self):
        msg = asrmsg.recog_results()
        self.asrpub.publish(msg)
        rospy.sleep(1)
        self.assertEquals(self.label, '')
        self.assertEquals(self.command, '')

    def test_no_string_asr_msg(self):
        attvalue = asrmsg.attribute_value()
        attvalue.atribute = ""
        attvalue.value = ""
        attvalue.confidence = 0.9

        msg = asrmsg.recog_results()
        msg.semantics.append(attvalue)
        self.asrpub.publish(msg)
        rospy.sleep(1)
        self.assertEquals(self.label, 'UNKNOWN', "label != UNKNOWN")
        self.assertEquals(self.command, 'UNKNOWN', "command != UNKNOWN")

    # def test_pose_label_inits_well(self):
    #     node = pln.PoseLabeler()
    #     # self.assertRaises(pl())




if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'pose_labeler_tests', PoseLabelerTestCase)
    # rostest.rosrun(PKG, 'pose_labeler_init', PoseLabelerInitTestCase)
    
    # import rosunit
    # rosunit.unitrun(PKG, 'PoseLabeler_oneequalsone', PoseLabelerTestCase)
