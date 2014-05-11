#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_instance_builder')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from func_utils import error_handler as eh
from func_utils import load_class
from param_utils import get_parameters, ParamNotFoundError

from pose_instance_builder.msg import PoseInstance
from std_msgs.msg import String


DEFAULT_NAME = 'instance_builder_node'
_NODE_PARAMS = ['builder_type', 'skeleton_topic']


def load_params(params):
    ''' loads parameters that will be used by the node '''
    try:
        for pname, pvalue in get_parameters(params):
            yield pvalue
    except ParamNotFoundError, e:
        logerr(e)
        raise

class InstanceBuilderNode():
    ''' Class that builds a dataset

        @keyword nodename: The name of the node
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                # action=self.shutdown,
                reraise=True):
            self.builder_type, skel_topic = load_params(_NODE_PARAMS)
            self.builder = load_class(self.builder_type)()
            self.skeleton_msg_type = self.builder.get_msg_class()
        
        rospy.Subscriber(skel_topic, self.skeleton_msg_type, self.skeleton_cb)
        rospy.Subscriber("pose_label", String, self.label_cb)
        self.label = ''

        # Publishers
        self.publisher = rospy.Publisher('pose_instance', PoseInstance)


    def skeleton_cb(self, msg):
        with eh(logger=loginfo, 
                log_msg='Instance not published.'):
            self.publisher.publish(self.builder.parse_msg(msg, self.label))

    def label_cb(self, label):
            self.label = label.data

    def run(self):
        rospy.spin()
    
    def shutdown(self):
        ''' Closes the node ''' 
        loginfo('Shutting down ' + rospy.get_name() + ' node.')
        

if __name__ == '__main__':
    try:
        node = InstanceBuilderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass