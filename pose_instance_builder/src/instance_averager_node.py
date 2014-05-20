#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_instance_builder')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

import pandas as pd

from func_utils import error_handler as eh
from func_utils import load_class
from param_utils import get_parameters, ParamNotFoundError
import circular_dataframe as cdf

from pose_instance_builder.msg import PoseInstance
from std_msgs.msg import String


_DEFAULT_NAME = 'instance_averager_node'
_NODE_PARAMS = ['builder_type', 'skeleton_topic']


def load_params(params):
    ''' Loads parameters that will be used by the node '''
    try:
        for pname, pvalue in get_parameters(params):
            yield pvalue
    except ParamNotFoundError, e:
        logerr(e)
        raise

# def _drop_older_rows(df, max_dflen):
#     ''' Drops n elements of the header where n = len(df) - max_dflen'''
#     if max_dflen <= 0:
#         return df
#     df_ = df
#     drop_counts = len(df_) - max_dflen
#     if drop_counts > 0:
#         df_ = df.drop(df.head(drop_counts).index)   # Drop drop_counts elems
#     return df_


# def append_instance(df, ins, max_dflen=50):
#     ''' Appends an an instance to the dataset 'df'. 

#         @param df: dataset at which the instance is added.
#         @type df: pandas.DataFrame
#         @param ins: Instance to be added to DataFrame
#         @type ins: numpy.ndarray (1D)
#         @param max_dflen: (Default 50) Max lenght of the dataframe
#                     If len(df) > 1 then drops first elem of df.
#         @return: a dataframe with the instance added to the end
#         @rtype: pandas.DataFrame
#     '''
#     if ins.ndim != 1:
#         raise ValueError("'ins' is not 1D. Shape: {}".format(ins.shape))

#     df_ = _drop_older_rows(df, max_dflen)
#     s = pd.Series(ins)
#     return df_.append(s, ignore_index=True)


class InstanceAveragerNode():
    ''' Node that processes skeleton messages and publishes them as instances
        
        It uses an L{InstanceBuilder} to convert the skeletons to instances.
        @keyword nodename: The name of the node
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', _DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        self.max_dflen = kwargs.get('max_dflen', 30)  # Max dataframe length
        self.averager = kwargs.get('averager', pd.DataFrame.mean)

        # Publishers and Subscribers
        rospy.Subscriber('pose_instance', PoseInstance, self.instance_cb)
        self.publisher = rospy.Publisher('averaged_pose', PoseInstance)
        self.df = pd.DataFrame()
        self.df_averaged = pd.DataFrame()

    def instance_cb(self, msg):
        instance = pd.Series(msg.instance, index=msg.columns)
        self.df = cdf.append_instance(self.df, instance, self.max_dflen)
        self.df_averaged = self.averager(self.df)
        pinstance = PoseInstance(instance=list(self.df_averaged.values),
                                 columns=list(self.df_averaged.index))
        self.publisher.publish(pinstance)
        
    def run(self):
        rospy.spin()
    
    def shutdown(self):
        ''' Closes the node ''' 
        loginfo('Shutting down ' + rospy.get_name() + ' node.')
        

if __name__ == '__main__':
    try:
        node = InstanceAveragerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass