#!/usr/bin/env python

import roslib; roslib.load_manifest('pose_tracker')
import rospy
from std_msgs.msg import String

import param_utils as pu
import pose_learner as pl

DEFAULT_NAME = 'pose_learner'
PARAMS = ('dataset_file', 'table_name', 'algorithm', 'parameter_grid', 
          'out_file')

class PoseLearnerNode():
    ''' Class that builds

        @keyword node_name: The name of the node
    '''
    def __init__(self, **kwargs):
        self.node_name = kwargs.get('node_name', DEFAULT_NAME)
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing " + self.node_name + " node...")
        
        self.load_parameters()
        # self.load_dataset(self.dataset)
        self.classif = pl.load_class_from_name(self.algorithm)
        
        rospy.Subscriber("~learn_this", String, self._learn_dataset_cb)
        self.ready_pub = rospy.Publisher('~classifier_ready', String)

    def load_parameters(self):
        ''' Loads the parameters needed by the node

            @Todo: Create variables using setattr() instead of doing it manually
            '''
        try:
            params = pu.get_parameters(PARAMS)
            self.dataset_file = params.next().value
            self.table_name = params.next().value
            self.algorithm = params.next().value
            self.parameter_grid = params.next().value
            self.out_file = params.next().value
        except:
            rospy.logfatal("Couldn't load Parameters")
            self.shutdown()

    def _learn_dataset_cb(self, dataset_file):
        self.load_dataset(dataset_file.data, self.table_name).fit().save_clf()

    def load_dataset(self, filename, table_name):
        self.dataset = pl.prepare_dataset(filename, table_name) \
                         .drop(pl.COLS_TO_CLEAN, axis=1)
        return self
        
    def fit(self):
        ''' Fits the classifier to the dataset data'''
        X, y = pl.df_to_Xy(self.dataset)
        self.classif = pl.fit_clf(X, y, 
                                param_grid=self.parameter_grid, 
                                model=self.classif)
        return self

    def save_clf(self):
        ''' Saves the best estimator to a file '''
        pl.save_clf(self.classif.best_estimator_, self.out_file)
        self.ready_pub.publish(self.out_file)
        return self

    def run(self):
        ''' Runs the node until shutdowns'''
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        ''' Closes the node ''' 
        rospy.loginfo('Shutting down ' + rospy.get_name() + ' node.')


if __name__ == '__main__':
    try:
        PoseLearnerNode().run()
    except rospy.ROSInterruptException:
        pass
