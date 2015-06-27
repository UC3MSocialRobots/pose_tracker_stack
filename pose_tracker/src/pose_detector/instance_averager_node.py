#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_tracker')
import rospy
from rospy import (loginfo, logerr, logfatal)

import pandas as pd
from scipy.stats import gmean as geometric_mean

from func_utils import error_handler as eh
# from func_utils import load_class
from param_utils import get_parameters, ParamNotFoundError
import circular_dataframe as cdf

from pose_instance_builder.msg import PoseInstance
# from std_msgs.msg import String


_DEFAULT_NAME = 'instance_averager_node'
_NODE_PARAMS = ['builder_type', 'skeleton_topic']


def __gmean(df):
    """Return geometric mean of a dataframe in form of a pandas.Series."""
    return pd.Series(geometric_mean(df), index=df.columns)


METHODS = {'mean': pd.DataFrame.mean,
           'median': pd.DataFrame.median,
           'gmean': __gmean}


def load_params(params):
    """Load parameters that will be used by the node."""
    try:
        for _, pvalue in get_parameters(params):
            yield pvalue
    except ParamNotFoundError, e:
        logerr(e)
        raise


class InstanceAveragerNode():

    """Node that processes skeleton messages and publishes them as instances.

    It uses an L{InstanceBuilder} to convert the skeletons to instances.
    """

    def __init__(self, **kwargs):
        """Constructor.

        Parameters
        ----------
        nodename : str (Optional)
            The name of the node
        """
        name = kwargs.get('node_name', _DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        # self.max_dflen = kwargs.get('max_dflen', 30)  # Max dataframe length
        # self.averager = kwargs.get('averager', pd.DataFrame.mean)

        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                action=self.shutdown, reraise=True):
            self.method, self.dflen = load_params(['averager_method',
                                                  'dataframe_length'])
            self.averager = METHODS.get(self.method, METHODS['mean'])

        # Publishers and Subscribers
        rospy.Subscriber('pose_instance', PoseInstance, self.instance_cb)
        self.publisher = rospy.Publisher('averaged_pose', PoseInstance)
        self.df = pd.DataFrame()
        self.df_averaged = pd.Series()

    def instance_cb(self, msg):
        """Callback. Publish a PoseInstance with averaged values."""
        instance = pd.Series(msg.instance, index=msg.columns)
        self.df = cdf.append_instance(self.df, instance, self.dflen)
        self.df_averaged = self.averager(self.df)
        pinstance = PoseInstance(instance=list(self.df_averaged.values),
                                 columns=list(self.df_averaged.index))
        self.publisher.publish(pinstance)

    def run(self):
        rospy.spin()

    def shutdown(self):
        """Close the node."""
        loginfo('Shutting down ' + rospy.get_name() + ' node.')


if __name__ == '__main__':
    try:
        node = InstanceAveragerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
