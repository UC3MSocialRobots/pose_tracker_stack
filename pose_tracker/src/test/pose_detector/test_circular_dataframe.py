PKG = 'pose_tracker'
import roslib; roslib.load_manifest(PKG)

# from pytest import mark
import unittest
import pandas as pd
from numpy import linspace

import circular_dataframe as cdf

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

    def test_zero_or_negative_max_lenghts_return_unmodified_df(self):
        for maxlength in (0, -1, -3, -10):
            for df in (self.df0, self.df1, self.df3, self.df5):
                self.assertTrue(all(cdf._drop_older_rows(df, maxlength) == df))

    def test_df_len_equals_max_len_return_unmodified_df(self):
        for df in (self.df0, self.df1, self.df3, self.df5):
            maxlen = len(df)
            self.assertTrue(all(cdf._drop_older_rows(df, maxlen) == df))

    def test_df_len_less_than_max_len_return_unmodified_df(self):
        for df in (self.df0, self.df1, self.df3, self.df5):
            maxlen = len(df) + 1
            self.assertTrue(all(cdf._drop_older_rows(df, maxlen) == df))

    # @mark.bench('cdf._drop_older_rows')
    def test_df_len_greater_than_max_len_drops_older_rows_of_df(self):
        for df in (self.df5, self.df10):
            for diff in (1, 3, 5):
                maxlen = len(df) - diff 
                new_df = cdf._drop_older_rows(df, maxlen)
                self.assertTrue(len(new_df) >= maxlen)
                self.assertTrue(all(new_df == df.tail(maxlen)))
            
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

    def test_raisesValueError_if_instance_is_not_1d(self):
        with self.assertRaises(ValueError):
            cdf.append_instance(self.df, self.df)

    def test_drops_older_instances_if_df_len_greater_than_maxlen(self):
        new_df = cdf.append_instance(self.df, self.s, 2)
        self.assertEqual(len(new_df), 2)
        self.assertTrue(all(new_df.ix[0] == self.df.ix[1]))
        
    def test_returns_new_df_with_appended_instance(self):
        new_df = cdf.append_instance(self.df, self.s, 2)
        self.assertEqual(len(new_df), 2)
        self.assertTrue(all(new_df.tail(1) == self.s))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_drop_older_rows', TestDropOlderRows)
    rosunit.unitrun(PKG, 'test_append_instances', TestAppendInstances)

