#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_instance_builder')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from func_utils import error_handler as eh

from pose_instance_builder.msg import PoseInstance


DEFAULT_NAME = 'instance_builder_node'

def load_instance_builder(name, module_name='instance_builder'):
    '''
        Factory that returns an instance of an InstanceBuilder

        Adapted from this SO Answer: http://stackoverflow.com/a/547867/630598
        @type name: string
        @param name: name of the InstanceBuilder class to instantiate
        @param module_name: (Default: 'instance_builder')
                            Module name where InstanceBuilder is defined
        @type module_name: string
        @return: an instance of module_name.Name.
        
        Example:
        --------
        >>> import instance_builder
        >>> builder = load_instance_builder('PiTrackerIBuilder')
        >>> isinstance(builder, instance_builder.PiTrackerIBuilder)
        ... True

    '''
    mod = __import__(module_name, fromlist=[name])
    klass = getattr(mod, name)
    return klass()



def load_params(self):
    ''' loads parameters that will be used by the node '''
    try:
        return rospy.get_param('~builder_type')
    except:
        logerr('Could not load parameter "~builder_type"')
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
                action=self.shutdown):
            self.builder_type = self.load_params()
            self.builder = load_instance_builder(self.builder_type)
            self.skeleton_msg = self.builder.get_msg_class()

        rospy.Subscriber("skeletons", self.skeleton_msg, self.skeleton_cb)

        # Publishers
        self.publisher = rospy.Publisher('~pose_instance', PoseInstance)
       
    def skeleton_cb(self, msg):
        self.publisher.publish(self.builder.parse_msg(msg))

    def run():
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