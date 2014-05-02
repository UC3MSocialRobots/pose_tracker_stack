#!/usr/bin/env python

import roslib; roslib.load_manifest('pose_tracker')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from pandas import Series

from func_utils import error_handler as eh
import param_utils as pu
import pose_learner as pl

import kinect.nite_skeleton_msg_utils as nsku

# Import message types
from pose_tracker.msg import PoseEstimated
from pose_tracker.srv import DatasetInfo
import kinect.msg as kin

DEFAULT_NAME = 'pose_estimator'
PARAMS = ('estimator_file', 'dataset_columns', 'drop_columns', 'labels' )


class PoseEstimatorNode():
    ''' Class that builds the node

        @keyword node_name: The name of the node
    '''
    def __init__(self, **kwargs):
        self.node_name = kwargs.get('node_name', DEFAULT_NAME)
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing " + self.node_name + " node...")
        
        with eh(action=self.shutdown):
            self.load_parameters()

         # Subscriber
        rospy.Subscriber("skeletons", kin.NiteSkeletonList, self.skeleton_cb)

        # Publishers
        self.publisher = rospy.Publisher('pose_estimated', PoseEstimated)


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
            raise

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

    def predict(self, instance):
        ''' 
            Predicts the output for an instance
            @TODO: fill this method
        '''
        return self.estimator.predict(instance)

    def predict_proba(self, instance):
        return self.estimator.predict_proba(instance)

    def _unpack_skeleton_msg(self, skel_msg):
        ''' Converts a NiteskeletonMsg to a list to a pandas.Series'''
        return Series(list(nsku.unpack_skeleton_msg(skel_msg)[1]), 
                      index=self.dataset_columns).drop(self.drop_columns,axis=1)

    def skeleton_cb(self, skels):
        with eh(logger=logwarn, low_msg='Could not estimate pose. '):
            pe_msg = PoseEstimated()
            pe_raw_instance = self._unpack_skeleton_msg(skels.skeletons[0]) 
            pe_msg.predicted_label_id = self.predict(pe_msg.raw_instance)
            pe_msg.predicted_label = self.labels[pe_msg.predicted_label_id]
            pe_msg.label_names = self.labels
            pe_msg.label_probas = self.predict_proba(msg.raw_instance)
    
            self.publisher.publish(pe_msg)

    def get_dataset_info(self):
        ''' Service client to get information of the dataset used for learning
        '''
        rospy.wait_for_service('dataset_info')
        with eh(loginfo, log_msg="Service call failed: ",
                errors=rospy.ServiceException):
            dinfo = rospy.ServiceProxy('dataset_info', DatasetInfo)
            response = dinfo()
            return response.filename, response.columns, response.labels

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
