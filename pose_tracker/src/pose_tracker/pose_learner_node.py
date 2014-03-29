#!/usr/bin/env python

import roslib; roslib.load_manifest('pose_tracker')
import rospy

DEFAULT_NAME = 'pose_learner'

class PoseLearnerNode():
    ''' Class that builds

        @keyword nodename: The name of the node
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', DEFAULT_NAME)

if __name__ == '__main__':
    try:
        node = PoseLearnerNode()
    except rospy.ROSInterruptException:
        pass
