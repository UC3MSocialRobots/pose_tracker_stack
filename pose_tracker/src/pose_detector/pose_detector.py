#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_tracker')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from operator import (gt, lt)
import itertools as it
import pandas as pd

from func_utils import error_handler as eh
from param_utils import load_params
import circular_dataframe as cdf

from pose_msgs.msg import (PoseInstance, JointVelocities)
from std_msgs.msg import String


_DEFAULT_NAME = 'instance_averager_node'
_NODE_PARAMS = ['builder_type', 'skeleton_topic']


class DatasetNotFullError(Exception): pass


def is_dataset_full(expected_len, dataset):
    if len(dataset != expected_len):
        raise DatasetNotFullError()


def do_if_predicate(predicate, action, action_args=[], action_kwargs={}):
    if predicate:
        action(*action_args, **action_kwargs)
    return predicate


def is_still(threshold, df):
    ''' Returns True if all joints in df are below threshold. 
        Otherwise: returns False '''
    return all(lt(threshold, df))


def is_moving(threshold, df):
    ''' Returns True if all joints in df are equal or above threshold. 
        Otherwise: returns False '''
    return all(gt(threshold, df))


# def next_caller(iterator):
#     return iterator.next(), iterator


class PoseDetectorNode():
    ''' Node that receives L{JointVelocities} and evaluates them to 
        publish wheter the user is moving or not.
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', _DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                action=rospy.signal_shutdown, reraise=True):
                self.dflen, self.threshold = load_params('dataframe_length', 
                                                         'movement_threshold')

        # Publishers and Subscribers
        rospy.Subscriber('/joint_velocities', JointVelocities, self.velocities_cb)
        rospy.Subscriber('/pose_instance', PoseInstance, self.instance_cb)
        self.publisher = rospy.Publisher('/user_pose', PoseInstance)
        
        self.velocities = pd.DataFrame()
        self.pose = pd.Series()
        self.detectors = it.cycle([is_still, is_moving])
        self.current_detector = self.detectors.next()
        

    def velocities_cb(self, msg):
        new_instance = pd.Series(msg.velocities, index=msg.columns)
        self.velocities = cdf.append_instance(self.velocities, new_instance)
        try:
            is_dataset_full(self.df)
            if self.current_detector(self.threshold, self.velocities):
                self.change_state()
                # TODO: ENSURE THAT I PUBLISH TO  DIFFERENT TOPICS
                self.publisher.publish(self.pose_instance)
        except DatasetNotFullError:
            pass
    
    def instance_cb(self, msg):
        ''' Stores the latest received L{PoseInstance} message '''
        self.pose_instance = msg
                
    def change_state(self, detectors):
        ''' Updates current detector and flushes the velocities dataset '''
        self.current_detector = self.detectors.next()
        self.velocities = pd.Dataframe()

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