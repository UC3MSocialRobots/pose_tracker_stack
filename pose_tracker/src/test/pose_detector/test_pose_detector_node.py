#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-05
'''
# from __future__ import division
PKG = 'pose_tracker'
import roslib
roslib.load_manifest(PKG)
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

import itertools as it
import unittest
# import numpy as np
# import pandas as pd
from func_utils import error_handler as eh
from param_utils import load_params
from pose_tracker.srv import Detector as DetectorSrv
from std_msgs.msg import Bool
from pose_msgs.msg import (PoseInstance, JointVelocities)
from pose_detector.pose_detector_node import (DatasetNotFullError,
                                              PoseDetectorNode, Detector,
                                              is_dataset_full,
                                              is_still, is_moving)

_NODE_PARAMS = ['dataframe_length', 'movement_threshold']

STILL_D = 'is_still_detector'
MOVING_D = 'is_moving_detector'
COLUMNS = list('ABCDEF')
MSG_LEN = len(COLUMNS)
POSE_INSTANCE = PoseInstance(columns=COLUMNS, instance=[0.0] * MSG_LEN)
STILL_MSG = JointVelocities(columns=COLUMNS, velocities=[0.1] * MSG_LEN)
MOVING_MSG = JointVelocities(columns=COLUMNS, velocities=[30.0] * MSG_LEN)
ALMOST_STILL_MSG = JointVelocities(columns=COLUMNS,
                                   velocities=[0.0, 30.0] * (MSG_LEN / 2))
ALMOST_MOVING_MSG = JointVelocities(columns=COLUMNS,
                                    velocities=[0.0, 30.0] * (MSG_LEN / 2))


class TestPoseDetectorCommon(unittest.TestCase):

    """Tests"""

    def __init__(self, *args):
        super(TestPoseDetectorCommon, self).__init__(*args)
        name = 'test_pose_detector'
        rospy.init_node(name)
        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                reraise=True):
            self.dflen, self.threshold = load_params(_NODE_PARAMS)
        # self.dflen = rospy.get_param('dataframe_length')
        # self.th = rospy.get_param('movement_threshold')

        self.instance_pub = rospy.Publisher('pose_instance', PoseInstance)
        self.velo_pub = rospy.Publisher('joint_velocities', JointVelocities)

        rospy.Subscriber('user_pose', PoseInstance, self.__user_pose_cb)
        rospy.Subscriber('user_moving', JointVelocities, self.__user_moving_cb)
        rospy.Subscriber('is_user_moving', Bool, self.__is_user_moving_cb)

        rospy.wait_for_service('detector')
        with eh(reraise=True):
            self.change_detector = rospy.ServiceProxy('detector', DetectorSrv)

    def setUp(self):
        self.received_pose = None
        self.user_moving = None
        self.is_user_moving = None
        rospy.sleep(1)
        self.instance_pub.publish(POSE_INSTANCE)

    def tearDown(self):
        pass

    def __user_pose_cb(self, msg):
        loginfo('Received message with user_still at: {}'.format(msg))
        self.received_pose = msg

    def __user_moving_cb(self, msg):
        loginfo('Received message with user_moving at: {}'.format(msg))
        self.user_moving = msg

    def __is_user_moving_cb(self, msg):
        loginfo('Received an is_user_moving predicate message: {}'.format(msg))
        self.is_user_moving = msg

    def publish_n(self, n, publisher, msg):
        for i in xrange(n):
            publisher.publish(msg)
            logerr("Published message to topic {}".format(publisher))
            print "Published message to topic {}".format(publisher)

    def fake_user_still(self):
        self.instance_pub.publish(POSE_INSTANCE)
        self.publish_n(self.dflen, self.velo_pub, STILL_MSG)
        # rospy.sleep(1)

    def fake_user_moving(self):
        self.instance_pub.publish(POSE_INSTANCE)
        self.publish_n(self.dflen, self.velo_pub, MOVING_MSG)
        # rospy.sleep(1)

    def set_detector(self, detector):
        response = self.change_detector(detector)
        if response.current_detector != detector:
            self.fail("Couldn't change detector")


class TestPoseDetectorNode(TestPoseDetectorCommon):

    @unittest.skip('TODO')
    def test_velocities_cb(self):
        self.fail("TODO")

    # @unittest.skip('TODO')
    def test_velocities_cb_publishes_an_instance_when_the_user_is_still(self):
        self.set_detector(STILL_D)
        self.fake_user_still()
        rospy.wait_for_message('user_pose', PoseInstance, timeout=6)
        self.assertEqual(self.received_pose, POSE_INSTANCE)

    # @unittest.skip('TODO')
    def test_velocities_cb_publishes_velos_when_the_user_starts_moving(self):
        self.set_detector(MOVING_D)
        self.fake_user_moving()
        rospy.wait_for_message('user_moving', JointVelocities, timeout=6)
        self.assertEqual(self.user_moving, MOVING_MSG)

    @unittest.skip('TODO')
    def test_Node_switches_detector_when_user_starts_or_stops_moving(self):
        ''' TODO: make a service in the node so others can know
            which detector is used each moment '''
        self.fail('TODO')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_pose_detector_node', TestPoseDetectorNode)
