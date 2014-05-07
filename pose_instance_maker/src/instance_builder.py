#!/usr/bin/python

import roslib; roslib.load_manifest('pose_instance_maker')
#import rospy
#from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from itertools import (izip, starmap)
from toolz import (concat, cons)

from pi_tracker.msg import Skeleton
from pose_instance_maker.msg import PoseInstance

def parse_label(label):
    ''' Parses the entered label.
        @return: str(label) or "UNKNOWN" if the label is empty.
        @rtype: str'''
    if not label:
        return 'UNKNOWN'
    return str(label)


class IBuilder():
    ''' Base class that defines the interface to that other builders
        should implement.
    '''
    def __init__(self, *args, **kwargs):
        pass

    def parse_msg(self, msg, label):
        ''' Gets msg and returns an instance message
            @param msg: The message to be converted to a PoseInstance
                        Each Builder will be able to process a single msg type
                        Thus, the type of msg will depend on the Builder.
            @return: The instance message filled with the msg data
            @rtype: pose_instance_maker.msg.PoseInstance
        '''
        pass

    def get_msg_class():
        ''' Should return the class of the skeleton message 
            that the builder is able to parse
        '''
        pass


class PiTrackerIBuilder():
    def __init__(self, *args, **kwargs):
        pass
    
    def parse_msg(self, msg, label):
        ''' Parses a pi_tracker.msg.Skeleton message and converts it to a 
            L{PoseInstance} message
            @name msg: The message to be parsed
            @type msg: pi_tracker.msg.Skeleton
            @return: The instance message already formatted
            @rtype: pose_instance_maker.msg.PoseInstance
        '''
        if not isinstance(msg, Skeleton):
            raise TypeError("Messge is not a pi_tracker/msg/Skeleton.msg")
        if not isinstance(label, str):
            raise TypeError("Label is not a string")

        msg_fields = izip(msg.position, msg.orientation, msg.confidence)
        instance = cons(msg.user_id, concat(starmap(self._parse, msg_fields)))
        return PoseInstance(columns=msg.name, 
                            label=parse_label(label),
                            instance=list(instance))

    def get_msg_class(self):
        return Skeleton

    def _parse(self, position, orientation, confidence):
        return (position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
                confidence)


class KinectIBuilder():
    def __init__():
        pass
    
    def parse_msg(self, msg, label):
        pass