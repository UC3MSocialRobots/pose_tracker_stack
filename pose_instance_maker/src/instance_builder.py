#!/usr/bin/python

import roslib; roslib.load_manifest('pose_instance_maker')
#import rospy
#from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from itertools import (chain, izip, starmap)
from more_itertools import flatten

from pi_tracker.msg import Skeleton
from pose_instance_maker.msg import PoseInstance

def parse_label(label):
    ''' Parses the entered label.
        @return: str(label) or "UNKNOWN" if the label is empty.'''
    if not label:
        return 'UNKNOWN'
    return str(label)

class PiTrackerIBuilder():
    def __init__(self, *args, **kwargs):
        pass
    
    def parse_msg(self, msg, label):
        ''' Parses a pi_tracker.msg.Skeleton message and converts it to a 
            L{PoseInstance} message
        '''
        if not isinstance(msg, Skeleton):
            raise TypeError("Messge is not a pi_tracker/msg/Skeleton.msg")
        if not isinstance(label, str):
            raise TypeError("Label is not a string")

        msg_fields = izip(msg.position, msg.orientation, msg.confidence)
        instance = chain(msg.user_id, flatten(starmap(self._parse, msg_fields)))
        return PoseInstance(columns=msg.name, 
                            label=parse_label(label),
                            instance=list(instance))

    def _parse(self, position, orientation, confidence):
        return (position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
                confidence)


class KinectIBuilder():
    def __init__():
        pass
    
    def parse_msg(self, msg, label):
        pass