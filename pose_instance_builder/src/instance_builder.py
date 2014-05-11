#!/usr/bin/python

import roslib; roslib.load_manifest('pose_instance_builder')
#import rospy
#from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from itertools import (izip, starmap)
from toolz import (concat, cons)

from pi_tracker.msg import Skeleton
from pose_instance_builder.msg import PoseInstance

# def parse_label(label):
#     ''' Parses the entered label.
#         @return: str(label) or "UNKNOWN" if the label is empty.
#         @rtype: str'''
   
#     return str(label)

def _parse_msg_preconditions(msg, msg_class, label):
    ''' Processes some preconditions for incoming messages '''
    if not isinstance(msg, msg_class):
         raise TypeError("Messge not a {}.msg".format(msg_class()))
    if not label:
        raise TypeError('Empty label')
    if not isinstance(label, str):
         raise TypeError("Label is not a string")
    if label == 'UNKNOWN':
        raise TypeError('"UNKNOWN" label')


class IBuilder():
    ''' Base class that defines the interface to that other builders
        should implement.
    '''
    def __init__(self, *args, **kwargs):
        pass

    def get_msg_class():
        ''' Should return the class of the skeleton message 
            that the builder is able to parse
        '''
        pass

    def parse_msg(self, msg, label):
        ''' Gets msg and returns an instance message
            @param msg: The message to be converted to a PoseInstance
                        Each Builder will be able to process a single msg type
                        Thus, the type of msg will depend on the Builder.
            @return: The instance message filled with the msg data
            @rtype: pose_instance_builder.msg.PoseInstance
        '''
        pass


class PiTrackerIBuilder():
    def __init__(self, *args, **kwargs):
        pass
    
    def get_msg_class(self):
        return Skeleton

    def parse_msg(self, msg, label):
        ''' Parses a pi_tracker.msg.Skeleton message and converts it to a 
            L{PoseInstance} message
            @name msg: The message to be parsed
            @type msg: pi_tracker.msg.Skeleton
            @return: The instance message already formatted
            @rtype: pose_instance_builder.msg.PoseInstance
            @raise TypeError if preconditions fail
        '''
        _parse_msg_preconditions(msg, self.get_msg_class(), label)
        
        msg_fields = izip(msg.position, msg.orientation, msg.confidence)
        instance = cons(msg.user_id, concat(starmap(self._parse, msg_fields)))
        return PoseInstance(columns=msg.name, 
                            label=str(label),
                            instance=list(instance))

    def _parse(self, position, orientation, confidence):
        return (position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
                confidence)


class KinectIBuilder():
    def __init__():
        pass
    
    def get_msg_class(self):
        return NiteSkeletonList

    def parse_msg(self, msg, label):
        _parse_msg_preconditions(msg, self.get_msg_class(), label)

        pass

