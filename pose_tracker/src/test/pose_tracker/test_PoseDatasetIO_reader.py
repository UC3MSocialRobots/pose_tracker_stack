#!/usr/bin/env python
PKG = 'pose_tracker'
import roslib; roslib.load_manifest(PKG)
import unittest
from mock import patch
import numpy as np
import pandas as pd

import pose_tracker.PoseDatasetIO as pdio


class PoseDatasetIOReaderTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(PoseDatasetIOReaderTestCase, self).__init__(*args)
    
    def setUp(self):
        cols = tuple('ABCDE')
        ind = ('fist','second', 'third','fourth')
        self.df = pd.DataFrame(np.linspace(1,20,20).reshape(4,5),
                                    columns=cols, index=ind)
        self.invalid_df= np.linspace(1,20,20).reshape(4,5)

        self.pose_dataset = \
            pdio.PoseDatasetIO(dataset='/tmp/reader_dataset', columns=cols)
        self.table_name = 'fake_table'

    def tearDown(self):
        pass

    @patch.object(pdio.pd, 'HDFStore')
    def _build_dataset(self, mock_hdfs):
        ''' Helper function to create fake datasets'''
        self.pose_dataset.create_dataset()
        self.pose_dataset.write(self.table_name, self.df)
        self.pose_dataset.fill_metadata()

    def test_get_metadata(self):
        self._build_dataset()
        self.pose_dataset.read_table(self.table_name)
        self.pose_dataset.store.get.assert_called_with(self.table_name)

    def test_read_table(self):
        self._build_dataset()
        self.pose_dataset.read_table(self.table_name)
        self.assertTrue(self.pose_dataset.store.get.called)
        
        

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_PoseDatasetIOReader', PoseDatasetIOReaderTestCase,
        coverage_packages=['../PoseDatasetIO.py',])

