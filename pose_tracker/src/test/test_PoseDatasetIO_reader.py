#!/usr/bin/env python
PKG = 'pose_tracker'
import roslib; roslib.load_manifest(PKG)
import unittest
from mock import patch
import numpy as np
import pandas as pd

import pose_tracker.PoseDatasetIO as pdio


@patch.object(pd.io.pytables.HDFStore, 'put')
class PoseDatasetIOReaderTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(PoseDatasetIOReaderTestCase, self).__init__(*args)
    
    def setUp(self):
        # cols = tuple('ABCDE')
        # ind = ('fist','second', 'third','fourth')
        # self.dataset = pd.DataFrame(np.linspace(1,20,20).reshape(4,5),
        #                             columns=cols, index=ind)
        # self.invalid_dataset = np.linspace(1,20,20).reshape(4,5)

        # self.writer = pdio.PoseDatasetIO(dataset='/tmp/writer', columns=cols)
        # self.writer.create_dataset()
        pass

    def tearDown(self):
        pass

    def _build_dataset(self):
        ''' Helper function to create fake datasets'''
        cols = tuple('ABCDE')
        ind = ('fist','second', 'third','fourth')
        self.dataset = pd.DataFrame(np.linspace(1,20,20).reshape(4,5),
                                    columns=cols, index=ind)
        self.invalid_dataset = np.linspace(1,20,20).reshape(4,5)

        self.ds_file = \
            pdio.PoseDatasetIO(dataset='/tmp/reader_dataset.h5', columns=cols)
        self.ds_file.create_dataset()

    def test_read_raises_IOError_if_file_not_found():
        pass

    def test_read_raises_IOError_if_dataset_not_created():
        pass
    
    def test_get_metadata_raises_IOError_if_no_metadata_has_been_entered():
        pass
    
    def test_get_metadata():
        pass
    
    def test_read(self, mock_put):
        pass

if __name__ == '__main__':
    # import rostest
    # rostest.rosrun(PKG, 'test_PoseDatasetIO', PoseDatasetIOInitTestCase)
       
    import rosunit
    # rosunit.unitrun(PKG, 'test_DateParser', DateParserTestCase)
    # rosunit.unitrun(PKG, 'test_PoseDatasetIO', PoseDatasetIOInitTestCase)
    rosunit.unitrun(PKG, 'test_PoseDatasetIOReader', PoseDatasetIOReaderTestCase,
        coverage_packages=['../PoseDatasetIO.py',])
    # rosunit.unitrun(PKG, 'test_PoseDatasetIOReader', PoseDatasetIOReaderTestCase)
