#!/usr/bin/python

import roslib
roslib.load_manifest('pose_instance_builder')
#import rospy
# from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from itertools import (chain, imap, izip, product, starmap)
from toolz import (concat, cons)

from pi_tracker.msg import Skeleton
from pose_msgs.msg import PoseInstance
from kinect.msg import NiteSkeletonList
import kinect.nite_skeleton_msg_utils as nsku


def _check_msg_preconditions(msg, msg_class, label):
    """ Processes some preconditions for incoming messages """
    # Check message type
    if not isinstance(msg, msg_class):
        raise TypeError("Messge not a {}.msg".format(msg_class()))
    # Check Labels
    if not label:
        raise TypeError('Empty label')
    if not isinstance(label, str):
        raise TypeError("Label is not a string")
    if label == 'UNKNOWN':
        raise TypeError('"UNKNOWN" label')


class IBuilder(object):

    """ Base class that defines the interface to that other builders
        should implement.
    """

    def __init__(self, *args, **kwargs):
        pass

    def get_msg_class(self):
        """ Should return the class of the skeleton message
            that the builder is able to parse
        """
        pass

    def parse_msg(self, msg, label):
        """ Gets msg and returns an instance message
            @param msg: The message to be converted to a PoseInstance
                        Each Builder will be able to process a single msg type
                        Thus, the type of msg will depend on the Builder.
            @return: The instance message filled with the msg data
            @rtype: pose_instance_builder.msg.PoseInstance
        """
        pass


class PiTrackerIBuilder(object):

    """ Instance Builder for skeletons coming from pi_tracker package"""

    def __init__(self, *args, **kwargs):
        pass

    def get_msg_class(self):
        return Skeleton

    def parse_msg(self, msg, label):
        """ Parses a pi_tracker.msg.Skeleton message and converts it to a
            L{PoseInstance} message
            @name msg: The message to be parsed
            @type msg: pi_tracker.msg.Skeleton
            @return: The instance message already formatted
            @rtype: pose_instance_builder.msg.PoseInstance
            @raise TypeError if preconditions fail
        """
        _check_msg_preconditions(msg, self.get_msg_class(), label)

        msg_fields = izip(msg.position, msg.orientation, msg.confidence)
        instance = cons(msg.user_id, concat(starmap(self._parse, msg_fields)))
        return PoseInstance(columns=msg.name,
                            label=str(label),
                            instance=list(instance))

    def _parse(self, position, orientation, confidence):
        return (position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
                confidence)


class KinectIBuilder(object):

    """ Instance Builder for skeletons coming from kinect package"""

    def __init__(self):
        self.joints = ['head', 'neck', 'torso',
                       'left_shoulder', 'left_elbow', 'left_hand',
                       'right_shoulder', 'right_elbow', 'right_hand',
                       'left_hip', 'left_knee', 'left_foot',
                       'right_hip', 'right_knee', 'right_foot']
        self.attribs = ['pos_x', 'pos_y', 'pos_z',
                        'orient_x', 'orient_y', 'orient_z', 'orient_w',
                        'pos_confidence', 'orient_confidence']
        self.header = ['user_id', 'stamp']
        self.cols = imap('_'.join, product(self.joints, self.attribs))
        self.cols = list(chain(self.header, self.cols))

    def get_msg_class(self):
        return NiteSkeletonList

    def parse_msg(self, msg, label):
        """ converts a NiteSkeletonList message to a PoseInstance message

            @raise: TypeError if preconditions fail
            @param msg: the NiteSkeletonList message
            @type msg: NiteSkeletonList
            @param label: the label to add to the PoseInstance message
            @type label: str
            @return: a PoseInstance message """
        self._check_parse_msg_preconditions(msg, label)
        skel = msg.skeletons[0]   # only parse the first skeleton
        instance, _ = nsku.unpack_skeleton_msg(skel)
        return PoseInstance(columns=self.cols,
                            label=str(label),
                            instance=list(instance))

    def _check_parse_msg_preconditions(self, msg, label):
        """ Helper method to check if all preconditions hold"""
        _check_msg_preconditions(msg, self.get_msg_class(), label)
        if not msg.skeletons:
            raise TypeError("Received a message with no skeletons")
