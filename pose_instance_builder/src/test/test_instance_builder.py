PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)


import unittest
from numpy import linspace
from itertools import repeat
from more_itertools import ncycles
from toolz import (concat, cons)

from instance_builder import (parse_label, PiTrackerIBuilder, KinectIBuilder)
from pi_tracker.msg import Skeleton
from geometry_msgs.msg import (Vector3, Quaternion)
from pose_instance_builder.msg import PoseInstance
        

class TestParseLabel(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestParseLabel, self).__init__(*args)
    
    def test_parse_label_returns_UNKNOWN_label(self):
        self.assertEqual('UNKNOWN', parse_label('UNKNOWN'))
        self.assertEqual('UNKNOWN', parse_label(None))
        self.assertEqual('UNKNOWN', parse_label([]))

    def test_parse_label(self):
        self.assertEqual('lalala', parse_label('lalala'))
        self.assertEqual('123', parse_label(123))
        self.assertEqual('123.0', parse_label(123.0))


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
        position = Vector3(*linspace(1,3,3))
        orientation = Quaternion(*linspace(1,4,4))
        return Skeleton(user_id=0, name=['c0', 'c1', 'c2', 'c3', 'c4'],
                        confidence=linspace(1,1,5),
                        position=list(repeat(position, 5)),
                        orientation=list(repeat(orientation,5)))

    def _make_instance(self):
        ''' Fill instance with:
                user_id, and 5 fake positions, orientaitons and confidences
            Position is a Vector3(x, y, z)
            Orientation is a Quaternion(x, y, z, w)
            Confidence is one field'''
        field_lens = (3,4,1)   # Lengths of Position, Orientation and Confidence
        inst = ncycles(concat(linspace(1,l,l) for l in field_lens), 5)
        inst = cons(0, inst)   # Insert user_id at the beginning of instance
        return PoseInstance(label    = 'lalala',
                            columns  = ['c0', 'c1', 'c2', 'c3', 'c4'],
                            instance = list(inst))
        
    def test_parse_msg_raises_TypeError_with_bad_peconditions(self):
        bad_msgs = ((None, 'lalala'), (self.skel, None),
                    (123, 'lalala'), (self.skel, 1234))
        for instance, label in bad_msgs:
            with self.assertRaises(TypeError):
                self.builder.parse_msg(instance, label)
        
    def test_parse_msg(self):
        self.assertEqual(self.instance, 
                         self.builder.parse_msg(self.skel, 'lalala'))


    def test_get_msg_class(self):
        skel = self.builder.get_msg_class()()
        self.assertTrue(isinstance(skel, Skeleton))


# class TestKinectIBuilder(unittest.TestCase):
#     """Tests"""
#     def __init__(self, *args):
#         super(TestKinectIBuilder, self).__init__(*args)

#     def setUp(self):
#         pass
        
#     def tearDown(self):
#         pass


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_ParseLabel', TestParseLabel)
    rosunit.unitrun(PKG, 'test_PiTrackerIBuiler', TestPiTrackerIBuiler)
    # rosunit.unitrun(PKG, 'test_KinectIBuilder', TestKinectIBuilder)
