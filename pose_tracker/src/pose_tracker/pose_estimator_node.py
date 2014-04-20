#!/usr/bin/env python

import roslib; roslib.load_manifest('pose_tracker')
import rospy
from std_msgs.msg import String

import param_utils as pu
import pose_learner as pl

DEFAULT_NAME = 'pose_estimator'
PARAMS = ('estimator_file', )

class PoseEstimatorNode():
    ''' Class that builds the node

        @keyword node_name: The name of the node
    '''
    def __init__(self, **kwargs):
        self.node_name = kwargs.get('node_name', DEFAULT_NAME)
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing " + self.node_name + " node...")
        
        self.load_parameters()

    def load_parameters(self):
        ''' Loads the parameters needed by the node.

            The node will acquire new attribs with the name of the loaded params
        '''
        try:
            params = pu.get_parameters(PARAMS)
            for p in params:
                pname = p.name.rsplit('/', 1)[-1]  # Get rid of param namespace
                setattr(self, pname, p.value)
        except:
            rospy.logfatal("Couldn't load Parameters: {}".format(list(params)))
            self.shutdown()

    def load_estimator(self, filename=None):
        ''' Loads an estimator from file

            @param filename: the file name of the file storing the estimator
                             Default: self.estimator_file
            @type filename: string
            @return: the estimator loaded from the file'''
        if not filename:
            filename = self.estimator_file
        self.estimator = pl.load_clf(filename)
        return self.estimator

    def predict(self):
        ''' 
            Predicts the output for an instance
            @TODO: fill this method
        '''
        pass

    def run(self):
        ''' Runs the node until shutdowns'''
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        ''' Closes the node ''' 
        rospy.loginfo('Shutting down ' + rospy.get_name() + ' node')


if __name__ == '__main__':
    try:
        PoseEstimatorNode().run()
    except rospy.ROSInterruptException:
        pass
