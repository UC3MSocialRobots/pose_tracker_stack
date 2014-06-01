#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-05
'''
from __future__ import division
PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

# import itertools as it
import numpy as np
import pandas as pd
from threading import Thread    # To run node in a separate thread

from joint_velocities_publisher import JointVelocitiesPublisher
from pose_instance_builder.msg import (PoseInstance, JointVelocities)

import unittest
# import numpy.testing 
from numpy.testing import assert_array_almost_equal as assert_arrAlmostEQ

class TestJointVelocitiesPublisher(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestJointVelocitiesPublisher, self).__init__(*args)
        name = 'test_joint_velocities_publisher'
        rospy.init_node(name)
                    
    def setUp(self):
        self.instances = pd.DataFrame(np.linspace(1,20,20).reshape(4,5),
                               columns=list('ABCDE'))
        self.recvd_velocities = pd.DataFrame()
        
        rospy.Subscriber('/joint_velocities', JointVelocities, self._velo_cb)
        self.publisher = rospy.Publisher('/pose_instance', PoseInstance)

    def tearDown(self):
        # del(self.node)
        pass

    def test_joint_velocities_publihser_advertises_joint_velocities_topic(self):
        rospy.sleep(0.3)  # Wait node is loaded
        self.assertTrue(any(self._search_topic_name('joint_velocities')))

    # @unittest.skip('Skipped')
    def test_publishes_velocities_when_receives_instances(self):
        # Pre-computed velocities
        expected_velocities = {1: np.array([0]*5),
                                    2: np.array([2.5]*5),
                                    3: np.array([3.33333333]*5),
                                    4: np.array([3.75]*5)}
        expected_velocities = pd.DataFrame(expected_velocities,
                                                columns=list('ABCDE'))
        
        self._publish_instances(self.instances)
        # rospy.sleep(1)
        self.assertEqual(len(self.recvd_velocities), 
                         len(expected_velocities),
                         "lengths missmatch.\nReceived:\n{}\nExpected:\n{}"
                         .format(self.recvd_velocities, expected_velocities))
        for i, row in expected_velocities.iterrows():
            self.assertSequenceEqual(self.recvd_velocities.iloc[i],
                                     expected_velocities.iloc[i])


    def _publish_instances(self, df):
        for i, row in df.iterrows():
            msg = PoseInstance(columns=row.index.tolist(),
                               instance=row.values.tolist())
            self.publisher.publish(msg)
            

    def _velo_cb(self, msg):
        velocities = pd.Series(msg.velocities, index=msg.columns)
        self.recvd_velocities = \
            self.recvd_velocities.append(velocities, ignore_index=True)
        

    def _search_topic_name(self, name, namespace='/'):
        ''' Returns a list of topics that contains name '''
        all_topics = rospy.get_published_topics(namespace=namespace)
        topic_names, topic_types = zip(*all_topics)
        return filter(lambda s: name in s, topic_names)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_joint_velocities_publisher',
                   TestJointVelocitiesPublisher)