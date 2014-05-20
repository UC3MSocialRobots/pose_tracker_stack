PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)


import unittest
import pandas as pd
from numpy import linspace
from itertools import (product, repeat)
from more_itertools import ncycles
from toolz import (concat, cons)

from instance_builder import (PiTrackerIBuilder, KinectIBuilder, 
                              _check_msg_preconditions)
from pi_tracker.msg import Skeleton
from kinect.msg import (NiteSkeletonList, NiteSkeleton, NiteSkeletonJoint)
from geometry_msgs.msg import (Vector3, Quaternion)
from pose_instance_builder.msg import PoseInstance
import kinect.nite_skeleton_msg_utils as nsku

from instance_averager_node import (append_instance, _drop_older_rows)

def _make_dataframe(nrows, columns):
        ncols = len(columns)
        array = linspace(1, nrows*ncols, nrows*ncols).reshape(nrows, ncols)
        return pd.DataFrame(array, columns=columns)

class TestDropOlderRows(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestDropOlderRows, self).__init__(*args)

    def setUp(self):
        df_lengthts = (0, 1, 3, 5, 10)
        self.cols = list('ABCDE')
        for l in df_lengthts:
            setattr(self, 'df{}'.format(l), _make_dataframe(l, self.cols))

        self.s = pd.Series(range(5), index=self.cols)
        
    def tearDown(self):
        pass

    def test_zero_or_negative_max_lenghts_do_not_modifies_df(self):
        for l in (0, -1, -3, -10):
            for df in (self.df0, self.df1, self.df3, self.df5):
                self.assertTrue(all(_drop_older_rows(df, l) == df))

    def test_df_len_equals_max_len_do_not_modifies_df(self):
        for df in (self.df0, self.df1, self.df3, self.df5):
            maxlen = len(df)
            self.assertTrue(all(_drop_older_rows(df, maxlen) == df))

    def test_df_len_less_than_max_len_do_not_modifies_df(self):
        for df in (self.df0, self.df1, self.df3, self.df5):
            maxlen = len(df) + 1
            self.assertTrue(all(_drop_older_rows(df, maxlen) == df))

    def test_df_len_greater_than_max_len_drops_older_rows_of_df(self):
        for df in (self.df5, self.df10):
            for diff in (1, 3, 5):
                maxlen = len(df) - diff 
                self.assertTrue(all(_drop_older_rows(df, maxlen) 
                                    == df.tail(maxlen)))
            
class TestAppendInstances(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestAppendInstances, self).__init__(*args)

    def setUp(self):
        self.cols = list('ABCDE')
        self.df = pd.DataFrame(linspace(1,10,10).reshape(2,5), columns=self.cols)
        self.s = pd.Series(range(5), index=self.cols)
        
    def tearDown(self):
        pass

    def test_append_instance_raisesTypeError_if_instance_is_not_1d(self):
        with self.assertRaises(TypeError):
            append_instance(self.df, self.df)

    def test_append_instance_returns_new_df_with_appended_instance(self):
        pass


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_drop_older_rows', TestDropOlderRows)
    rosunit.unitrun(PKG, 'test_append_instances', TestAppendInstances)

