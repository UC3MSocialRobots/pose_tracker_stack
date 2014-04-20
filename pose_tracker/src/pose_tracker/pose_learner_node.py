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

        self.classif = pl.load_class_from_name(self.algorithm)()
        rospy.loginfo("Classifier loaded: {}".format(self.classif))
        
        rospy.Subscriber("~learn_this", String, self._learn_dataset_cb)
        self.ready_pub = rospy.Publisher('~classifier_ready', String)

        self.load_dataset(self.dataset_file, self.table_name).fit().save_clf()        

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

    def _learn_dataset_cb(self, dataset_file):
        try:
            self.load_dataset(dataset_file.data, self.table_name)
            self.fit().save_clf()
        except IOError:
            pass

    def load_dataset(self, filename, table_name):
        try:
            self.dataset = pl.prepare_dataset(filename, table_name) \
                         .drop(pl.COLS_TO_CLEAN, axis=1)
            return self
        except IOError:
            rospy.logerror("Couldn't load dataset {}".format(filename))
            raise
        
    def fit(self):
        ''' Fits the classifier to the dataset data'''
        X, y = pl.df_to_Xy(self.dataset)
        self.classif = pl.fit_clf(X, y, 
                                param_grid=self.parameter_grid, 
                                estimator=self.classif)
        return self

    def save_clf(self):
        ''' Saves the best estimator to a file '''
        pl.save_clf(self.classif.best_estimator_, self.out_file)
        self.ready_pub.publish(self.out_file)
        rospy.loginfo("Classifier saved to: {}".format(self.out_file))
        return self

    def run(self):
        ''' Runs the node until shutdowns'''
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        ''' Closes the node ''' 
        rospy.loginfo('Shutting down ' + rospy.get_name() + ' node')


if __name__ == '__main__':
    try:
        PoseLearnerNode().run()
    except rospy.ROSInterruptException:
        pass
