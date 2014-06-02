#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_instance_builder')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

import pandas as pd

from func_utils import error_handler as eh
from param_utils import get_parameters, ParamNotFoundError
import circular_dataframe as cdf

from pose_msgs.msg import (PoseInstance, JointVelocities)

_DEFAULT_NAME = 'joint_velocities_publisher'


def load_params(params):
    ''' Loads parameters that will be used by the node '''
    try:
        for pname, pvalue in get_parameters(params):
            yield pvalue
    except ParamNotFoundError, e:
        logerr(e)
        raise


def calc_velocities(df):
    ''' Function that calculates velocities of an 
        input Dataframe with PoseInstances '''
    if not all(df.shape):
        raise ValueError("DataFrame is empty")
    return (df.iloc[-1].values - df.iloc[0].values) / len(df)


class JointVelocitiesPublisher():
    ''' Node that calculates joint velocities from L{PoseInstance} messages
        
        Note velocity is calculated not by time, but considering the number
        of received pose num_instances

        Loaded Parameters:
        ------------------
        'num_instances': num of pose_instances to calculate the velocity
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', _DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                action=self.shutdown, reraise=True):
                self.df_length = load_params(['num_instances']).next()

        # Publishers and Subscribers
        rospy.Subscriber('/pose_instance', PoseInstance, self.instance_cb)
        self.publisher = rospy.Publisher('/joint_velocities', JointVelocities)
        self.df = pd.DataFrame()

    def instance_cb(self, msg):
        instance = pd.Series(msg.instance, index=msg.columns)
        self.df = cdf.append_instance(self.df, instance, self.df_length)
        
        with eh(logger=loginfo, errors=ValueError,
                log_msg='Empty DataFrame. Velocities not published'):
            vels = calc_velocities(self.df)
            velocities = JointVelocities(velocities=vels.tolist(),
                                         columns=self.df.columns.tolist())
            self.publisher.publish(velocities)
        
    def run(self):
        rospy.spin()
    
    def shutdown(self):
        ''' Closes the node ''' 
        loginfo('Shutting down ' + rospy.get_name() + ' node.')
        

if __name__ == '__main__':
    try:
        node = JointVelocitiesPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass