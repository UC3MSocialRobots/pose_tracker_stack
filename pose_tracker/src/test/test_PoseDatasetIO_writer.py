#!/usr/bin/env python
PKG = 'pose_tracker'
import roslib; roslib.load_manifest(PKG)
import unittest
from mock import patch
import numpy as np
import pandas as pd

import pose_tracker.PoseDatasetIO as pdio


@patch.object(pd.io.pytables.HDFStore, 'put')
class PoseDatasetIOWriterTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(PoseDatasetIOWriterTestCase, self).__init__(*args)

    def setUp(self):
        cols = tuple('ABCDE')
        ind = ('fist','second', 'third','fourth')
        self.dataset = pd.DataFrame(
            np.linspace(1,20,20).reshape(4,5), columns=cols, index=ind)
        self.invalid_dataset = np.linspace(1,20,20).reshape(4,5)

        self.writer = pdio.PoseDatasetIO(dataset='/tmp/writer', columns=cols)
        self.writer.create_dataset()
        

    def tearDown(self):
        pass

    def test_init_with_bad_args(self, mock_put):
        bad_args = {'not_valid_param': 'nono',
                    'another invalid param': tuple('qwerty')}
        self.assertRaises(Exception, pdio.PoseDatasetIO, (), "PoseDatasetIO should not init /wo parameters!")
        self.assertRaises(Exception, pdio.PoseDatasetIO, bad_args, "Bad params test failed")

    def test_init_with_bad_types(self, mock_put):
        bad_types_1 = { 'dataset': 135,
                        'columns': 'a_string_should_not_be_valid'}
        bad_types_2 = { 'dataset': 'datasetname',
                        'columns': 'a_string_should_not_be_valid'}                 
        bad_types_3 = { 'dataset': ('a', 'b', 'c'),
                        'columns': ('a', 'b', 'c')}
        self.assertRaises(Exception, pdio.PoseDatasetIO, bad_types_1, "Bad types test 1 failed")
        self.assertRaises(Exception, pdio.PoseDatasetIO, bad_types_2, "Bad types test 2 failed")
        self.assertRaises(Exception, pdio.PoseDatasetIO, bad_types_3, "Bad types test 3 failed")
    
    def test_init_with_good_args(self, mock_put) :
        correct_args = {'dataset': 'dataset_name',
                        'columns': tuple('abcde')}
        try:
            pdio.PoseDatasetIO(**correct_args)
        except:
            self.fail("Should not have failed with correct arguments")

    @patch('pandas.Series')
    def test_fill_metadata(self, mock_pdSeries, mock_put):
        data_index = ('creator', 'date','descr')
        def_creator = 'Anonymous'
        fake_date = '2013-11-14 18:45'
        def_date = '1900-01-01 00:00'
        def_description =   "Default description. " \
                            "User forgot to add it when created " \
                            "the dataset."
        def_args = (def_creator, def_date, def_description)

        good_kwargs = {'creator': 'victor', 
                       'date': fake_date,
                       'descr':  'a description of the dataset'}
        good_args = tuple([good_kwargs[k] for k in sorted(good_kwargs)])

        self.writer.fill_metadata()
        mock_pdSeries.assert_called_with(def_args, index=data_index)
        mock_put.assert_called_with('description', 
                                        pd.Series(def_args, index=data_index))

        self.writer.fill_metadata(**good_kwargs)
        mock_pdSeries.assert_called_with(good_args, index=data_index)
        mock_put.assert_called_with('description', 
                                        pd.Series(good_args, index=data_index))


    @patch('pandas.Series')
    def test_fill_invalid_metadata(self, mock_pdSeries, mock_put):
        data_index = ('creator', 'date','descr')
        def_creator = 'Anonymous'
        def_date = '1900-01-01 00:00'
        def_description =   "Default description. " \
                            "User forgot to add it when created " \
                            "the dataset."
        def_args = (def_creator, def_date, def_description)

        bad_kwargs = {  'aaa': 111,
                        'bbb': 222,
                        'ccc': 'description of a bad argument '}
        
        self.writer.fill_metadata(**bad_kwargs)
        mock_pdSeries.assert_called_with(def_args, index=data_index)
        mock_put.assert_called_with(
            'description', pd.Series(def_args, index=data_index))

    # def test_columns_when_write(self, mock_put):
    #     pass

    def test_write_invalid_data_raises_an_exception(self, mock_put):
        not_a_string = 12345
        # empty_list = []
        empty_df = pd.DataFrame()
        self.assertRaises(TypeError, 
                          self.writer.write, (not_a_string, self.dataset))
        self.assertRaises(TypeError, 
                          self.writer.write, ('test_table', self.invalid_dataset))
        self.assertRaises(ValueError, 
                          self.writer.write, *('test_table', empty_df))

    def test_write(self, mock_put):
        # args = (self.dataset, 'test_table')
        args = ('test_table', self.dataset)
        self.writer.write(*args)
        self.assertTrue(mock_put.called)
        mock_put.assert_called_with(*args)

        kwargs = {}
        kwargs['append'] = True
        self.writer.write(*args, **kwargs)
        mock_put.assert_called_with(*args, **kwargs)

        kwargs['table'] = True
        self.writer.write(*args, **kwargs)
        mock_put.assert_called_with(*args, **kwargs)

        kwargs['append'] = False
        kwargs['table'] = False
        self.writer.write(*args, **kwargs)
        mock_put.assert_called_with(*args, **kwargs)
    
    
if __name__ == '__main__':
    # import rostest
    # rostest.rosrun(PKG, 'test_PoseDatasetIO', PoseDatasetIOInitTestCase)
       
    import rosunit
    # rosunit.unitrun(PKG, 'test_DateParser', DateParserTestCase)
    # rosunit.unitrun(PKG, 'test_PoseDatasetIO', PoseDatasetIOInitTestCase)
    rosunit.unitrun(PKG, 'test_PoseDatasetIOWriter', PoseDatasetIOWriterTestCase,
        coverage_packages=['../PoseDatasetIO.py',])
    # rosunit.unitrun(PKG, 'test_PoseDatasetIOReader', PoseDatasetIOReaderTestCase)
