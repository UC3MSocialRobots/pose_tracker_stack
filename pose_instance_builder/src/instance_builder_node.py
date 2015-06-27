#!/usr/bin/env python
"""
Node that processes skeleton messages and publishes them as instances.

:author: Victor Gonzalez
"""

import roslib
roslib.load_manifest('pose_instance_builder')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from func_utils import error_handler as eh
from func_utils import load_class
from param_utils import get_parameters, ParamNotFoundError

from pose_msgs.msg import PoseInstance
from std_msgs.msg import String


_DEFAULT_NAME = 'instance_builder_node'
_NODE_PARAMS = ['builder_type', 'skeleton_topic']


def load_params(params):
    """Loads parameters that will be used by the node."""
    try:
        for _, pvalue in get_parameters(params):
            yield pvalue
    except ParamNotFoundError as e:
        logerr(e)
        raise


class InstanceBuilderNode(object):

    """
    Node that processes skeleton messages and publishes them as instances.

    It uses an L{InstanceBuilder} to convert the skeletons to instances.
    @keyword nodename: The name of the node
    """

    def __init__(self, **kwargs):
        """Constructor."""
        name = kwargs.get('node_name', _DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                action=self.shutdown, reraise=True):
            self.builder_type, self.skel_topic = load_params(_NODE_PARAMS)
            self.builder = load_class(self.builder_type)()
            self.skeleton_msg_type = self.builder.get_msg_class()
            loginfo("Using Instance Builder: {}".format(self.builder_type))

        rospy.Subscriber(self.skel_topic, self.skeleton_msg_type, self.skel_cb)
        loginfo("Subscribed to topic {}".format(self.skel_topic))
        rospy.Subscriber("pose_label", String, self.label_cb)
        self.label = ''

        # Publishers
        self.publisher = rospy.Publisher('pose_instance', PoseInstance)

    def skel_cb(self, msg):
        """Callback for skeleton msgs.

        Publishes an instance from a skeelton msg."""
        with eh(logger=loginfo,
                log_msg='Instance not published. '):
            pose_instance = self.builder.parse_msg(msg, self.label)
            self.publisher.publish(pose_instance)

    def label_cb(self, label):
        """Store the received label."""
        self.label = label.data

    def run(self):
        """Ros spin."""
        rospy.spin()

    def shutdown(self):
        """Close the node."""
        loginfo('Shutting down ' + rospy.get_name() + ' node.')


if __name__ == '__main__':
    try:
        node = InstanceBuilderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
