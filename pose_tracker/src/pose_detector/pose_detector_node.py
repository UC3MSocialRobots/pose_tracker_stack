#!/usr/bin/env python
import roslib
roslib.load_manifest('pose_tracker')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)
from rospy import (Publisher, Subscriber)

# from operator import (gt, lt)
from collections import namedtuple
# imitertools as it
from itertools import cycle
import pandas as pd

from func_utils import error_handler as eh
from param_utils import load_params
import circular_dataframe as cdf

from pose_msgs.msg import (PoseInstance, JointVelocities)
from std_msgs.msg import Bool


class DatasetNotFullError(Exception):
    pass


def is_dataset_full(expected_len, dataset):
    if len(dataset) != expected_len:
        raise DatasetNotFullError()


def is_still(threshold, df):
    ''' Returns True if all joints in df are below threshold.
        Otherwise, returns False '''
    return df.lt(threshold).values.all()


def is_moving(threshold, df):
    ''' Returns True if all joints in df are equal or above threshold.
        Otherwise, returns False '''
    return df.gt(threshold).values.all()


def make_joint_velocities_msg(self, velocities):
    ''' Returns a L{JointVelocities} msg from the entered velocities dataframe
    '''
    velos = []
    try:
        velos = velocities.iloc[-1].values
    except:
        velos = [0] * len(velocities.columns)
    return JointVelocities(columns=velocities.columns, velocities=velos)


# def next_caller(iterator):
#     return iterator.next(), iterator
_DEFAULT_NAME = 'pose_detector_node'
_NODE_PARAMS = ['dataframe_length', 'movement_threshold']

Detector = namedtuple('Detector', ['detector', 'publisher', 'msg'])


class PoseDetectorNode():

    ''' Node that receives L{JointVelocities} and evaluates them to
        publish wheter the user is moving or not.

        Publishes a L{PoseInstance} to /user_pose if the user L{is_still}
        Publishes a L{JointVelocities} to /user_moving if the user L{is_moving}
        Note that it only publishes if there has been a change

        Example:
        --------
            1. User is moving
            2. User stops. --> publication to /user_pose
            3. User keeps stoped.
            4. User starts moving --> publication to /user_moving
            5. User stops again --> publication to /user_pose
    '''

    def __init__(self, **kwargs):
        name = kwargs.get('node_name', _DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                reraise=True):
            self.dflen, self.threshold = load_params(_NODE_PARAMS)

        ### Publishers and Subscribers
        Subscriber('pose_instance', PoseInstance, self.instance_cb)
        Subscriber('joint_velocities', JointVelocities, self.velo_cb)
        self.__pose_pub = Publisher('user_pose', PoseInstance, latch=True)
        self.__is_moving_pub = Publisher('is_user_moving', Bool, latch=True)
        self.__moving_pub = Publisher('user_moving',
                                      JointVelocities, latch=True)

        self.velocities = pd.DataFrame()
        self.pose_instance = PoseInstance()

        self.__build_detectors()

    def __build_detectors(self):
        ''' Helper method to create the detectors
            used to known wheter the user is still or moving'''
        self._still_detector = \
            Detector(is_still, self.__pose_publisher, self.pose_instance)
        self._moving_detector = \
            Detector(is_moving, self.__velo_publisher, self.velocities)

        self.detectors = cycle([self._still_detector, self._moving_detector])
        self.current_detector = self.detectors.next()

    def instance_cb(self, msg):
        ''' Stores the latest received L{PoseInstance} message '''
        logwarn("Instance Received: {}".format(msg))
        self.pose_instance = msg

    def velo_cb(self, msg):
        ''' Callback called when L{JointVelocities} msg is received'''
        logwarn("User is moving at velocity:\n{}".format(msg))
        self._add_msg_to_dataset(msg)
        self.check_dataset()

    def __publish_is_moving_predicate(self, predicate):
        ''' Publishes a predicate indicating wether the user is moving '''
        logwarn('Publishing Is User Moving: {}'.format(predicate))
        self.__is_moving_pub.publish(Bool(predicate))

    def __pose_publisher(self, pose_instance):
        ''' Helper method that publishes the user pose
            and a predicate indicating that the user is not moving'''
        self.__pose_pub.publish(pose_instance)
        logwarn('Published user pose:\n{}'.format(pose_instance))
        self.__publish_is_moving_predicate(False)

    def __velo_publisher(self, velocities):
        ''' Helper method that publishes the last velocities instance from
            the L{PoseDetectorNode.velocities} DataFrame
            and a predicate indicating the user is moving'''
        msg = make_joint_velocities_msg(velocities)
        self.__moving_pub.publish(msg)
        logwarn('Published user moving: {}'.format(msg))
        self.__publish_is_moving_predicate(True)

    def _add_msg_to_dataset(self, msg):
        new_instance = pd.Series(msg.velocities, index=msg.columns)
        self.velocities = cdf.append_instance(self.velocities, new_instance)

    def check_dataset(self):
        ''' Uses current detector to check if detector condition holds.
            If so, then publishes a message and changes the detector'''
        try:
            is_dataset_full(self.dflen, self.velocities)
        except DatasetNotFullError:
            return
        if self.current_detector.detector(self.threshold, self.velocities):
            self.change_detector(self.detectors)
            self.current_detector.publisher(self.current_detector.msg)
        return self

    def change_detector(self, detectors):
        ''' Updates current detector and flushes the velocities dataset '''
        self.current_detector = detectors.next()
        self.velocities = pd.DataFrame()
        return self

    def run(self):
        rospy.spin()

    def shutdown(self):
        ''' Closes the node '''
        loginfo('Shutting down ' + rospy.get_name() + ' node.')


if __name__ == '__main__':
    try:
        node = PoseDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
