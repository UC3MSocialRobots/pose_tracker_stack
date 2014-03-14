#!/usr/bin/env python
PKG = 'pose_tracker'
import roslib; roslib.load_manifest(PKG)
import unittest
# from mock import patch
# import numpy as np
# import pandas as pd
from itertools import chain

import pose_tracker.PoseDatasetIO as pdio


class DateParserTestCase(unittest.TestCase):
    """Tests for the date_parser included in PoseDatasetIO module"""
    def __init__(self, *args):
        super(DateParserTestCase, self).__init__(*args)

    def setUp(self):
        pass

    def tearDown(self):
        pass

        #assert creation date is well formatted (YYYY-MM-DD)        
    def test_valid_dates(self):
        valid_dates = ( '1900-01-01 00:00:00', '2013-11-14 18:45:23',
                        '2013-11-14 18:45', '2013-11-14')
        semi_valid_dates = ('2014-2-2','2015-02-02 2:12', 
                            '2015-02-02 02:2', '2015-02-02 02:02:1')
        parsed_sv_dates = ('2014-02-02','2015-02-02 02:12', 
                            '2015-02-02 02:02', '2015-02-02 02:02:01')
        
        for vd in chain(valid_dates, semi_valid_dates):
            try:
                pdio.parse_date(vd)
            except:
                self.fail("Should have not thrown an excpetion "
                          "with date: {}".format(vd))
        for vd in valid_dates:
            parsed_date = pdio.parse_date(vd)
            self.assertEqual(vd, parsed_date, 
                "Entered valid date {} differs from parsed {}".format(vd, 
                                                                parsed_date))
        for i, svd in enumerate(semi_valid_dates):
            parsed_date = pdio.parse_date(svd)
            self.assertEqual(parsed_sv_dates[i], parsed_date, 
                "Entered valid date {} differs from parsed {}".format(svd, 
                                                                parsed_date))

    def test_invalid_dates(self):
        bad_formated_dates = (  '14-11-2013', 'aaa-bbb-ccc', 'aabbcc', 
                                111222333, -103124.000, '2013-15-50', 
                                '2013-15-50 70:22', '2015-02-02 02:102:02',
                                '2015-02-02 02:02:99', '0001-01-01 00:00')
        for bad_date in bad_formated_dates:
            print bad_date
            self.assertRaises(ValueError, pdio.parse_date, bad_date)

if __name__ == '__main__':
    # import rostest
    # rostest.rosrun(PKG, 'test_PoseDatasetIO', PoseDatasetIOInitTestCase)
       
    import rosunit
    rosunit.unitrun(PKG, 'test_DateParser', DateParserTestCase)
    
