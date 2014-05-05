PKG = 'pose_instance_maker'
import roslib; roslib.load_manifest(PKG)


import unittest
from numpy.random import rand as arand

from instance_builder import (parse_label, PiTrackerIBuilder, KinectIBuilder)
from pi_tracker.msg import Skeleton
from geometry_msgs.msg import (Vector3, Quaternion)
from pose_instance_maker.msg import PoseInstance
        

class TestParseLabel(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestParseLabel, self).__init__(*args)
    
    def test_parse_label_returns_UNKNOWN_label(self):
        self.assertEqual('lalala', parse_label('lalala'))
        self.assertEqual('123', parse_label(123))
        self.assertEqual('123.0', parse_label(123.0))
        self.assertEqual('UNKNOWN', parse_label(None))
        self.assertEqual('UNKNOWN', parse_label([]))


class TestPiTrackerIBuiler(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestPiTrackerIBuiler, self).__init__(*args)
        
    def setUp(self):
        self.builder = PiTrackerIBuilder()
        self.skel = self._make_skeleton_msg()
        self.instance = self._make_instance()

    def tearDown(self):
        pass
        
    def _make_skeleton_msg(self):
        ''' Helper method to produce facke Skeleton msgs '''
        skel = Skeleton(user_id=0,
                        name=['c0', 'c1', 'c2', 'c3', 'c4'],
                        confidence=map(float, xrange(5)))
        for _ in xrange(5):
            skel.position.append(Vector3(*map(float, xrange(3))))
            skel.orientation.append(Quaternion(*map(float, xrange(4))))
        return skel

    def _make_instance(self):
        instance = PoseInstance(label='lalala',
                                columns=['c0', 'c1', 'c2', 'c3', 'c4'])
        for i in xrange(5):
            for j in xrange(3):             # position: Vector3
                instance.columns.append(j)   
            for j in xrange(4):             # orientation: Quaternion
                instance.columns.append(j)   
            instance.columns.append(i)      # confidence
        return instance

    def test_parse_msg_peconditions(self):
        with self.assertRaises(TypeError):
            self.builder.parse_msg(None, 'lalala')
        with self.assertRaises(TypeError):
            self.builder.parse_msg(self.skel, None)
        with self.assertRaises(TypeError):
            self.builder.parse_msg(123, 'lalala')
        with self.assertRaises(TypeError):
            self.builder.parse_msg(self.skel, 1234)

    def test_parse_msg(self):
        self.assertEqual(self.instance, 
                         self.builder.parse_msg(self.skel, 'lalala'))


class TestKinectIBuilder(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestKinectIBuilder, self).__init__(*args)

    def setUp(self):
        pass
        
    def tearDown(self):
        pass


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_TestParseLabel', TestParseLabel)
    rosunit.unitrun(PKG, 'test_TestPiTrackerIBuiler', TestPiTrackerIBuiler)
    rosunit.unitrun(PKG, 'test_KinectIBuilder', TestKinectIBuilder)
