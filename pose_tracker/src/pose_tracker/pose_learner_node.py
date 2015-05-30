#!/usr/bin/env python

import roslib
roslib.load_manifest('pose_tracker')
import rospy
from rospy import (logerr, logfatal)
from std_msgs.msg import String

import param_utils as pu
from func_utils import load_class
import pose_learner as pl
from func_utils import error_handler as eh

DEFAULT_NAME = 'pose_learner'
PARAMS = ('dataset_file', 'table_name', 'algorithm', 'parameter_grid',
          'out_file', 'drop_columns')


class PoseLearnerNode():

    """ Class that builds

        @keyword node_name: The name of the node
    """

    def __init__(self, **kwargs):
        self.node_name = kwargs.get('node_name', DEFAULT_NAME)
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing " + self.node_name + " node...")

        with eh(logger=logfatal, action=self.shutdown, reraise=True):
            self.load_parameters()
            self.classif = load_class(self.algorithm)()
            rospy.loginfo("Classifier loaded: {}".format(self.classif))
            self.load_dataset(self.dataset_file, self.table_name)
            self.fit().save_clf()

        rospy.Subscriber("~learn_this", String, self._learn_dataset_cb)
        self.ready_pub = rospy.Publisher('~classifier_ready', String)

    def load_parameters(self):
        """
        Load the parameters needed by the node.

        The node will acquire new attribs with the name of the loaded params
        """
        try:
            params = pu.get_parameters(PARAMS)
            for p in params:
                pname = p.name.rsplit('/', 1)[-1]  # Get rid of param namespace
                setattr(self, pname, p.value)
        except:
            logerr("Couldn't load Parameters: {}".format(list(params)))
            raise

    def _learn_dataset_cb(self, dataset_file):
        """Callback to learn dataset."""
        with eh(reraise=False):
            self.load_dataset(dataset_file.data, self.table_name)
            self.fit()
            self.save_clf()

    def load_dataset(self, filename, table_name):
        """Load dataset from filename."""
        with eh(logger=logerr, errors=IOError, reraise=True,
                log_msg="Couldn't load dataset {}".format(filename)):
            self.dataset = pl.prepare_dataset(filename, table_name) \
                .drop(self.drop_columns, axis=1)
        return self

    def fit(self):
        """Fit the classifier to the dataset data."""
        X, y = pl.df_to_Xy(self.dataset)
        self.classif = pl.fit_clf(X, y,
                                  param_grid=self.parameter_grid,
                                  estimator=self.classif)
        return self

    def save_clf(self):
        """Save the best estimator to a file."""
        pl.save_clf(self.classif.best_estimator_, self.out_file)
        self.ready_pub.publish(self.out_file)
        rospy.loginfo("Classifier saved to: {}".format(self.out_file))
        return self

    def run(self):
        """Run the node until shutdowns."""
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        """Close the node."""
        rospy.loginfo('Shutting down ' + rospy.get_name() + ' node')


if __name__ == '__main__':
    try:
        PoseLearnerNode().run()
    except rospy.ROSInterruptException:
        pass
