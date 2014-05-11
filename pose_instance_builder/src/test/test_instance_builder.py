PKG = 'pose_instance_builder'
import roslib; roslib.load_manifest(PKG)


import unittest
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

class TestPreconditions(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestPreconditions, self).__init__(*args)

    def setUp(self):
        self.pi_tracker_msg = Skeleton()
        self.pi_tracker_msg_type = Skeleton
        self.nite_skel_msg = NiteSkeletonList()
        self.nite_skel_msg_type = NiteSkeletonList

    def tearDown(self):
        pass

    def test_check_msg_preconditions_raises_TypeError_with_bad_msgs(self):
        label = 'lalala'
        msgs = [None, [], 1234]
        types = [self.pi_tracker_msg_type, self.nite_skel_msg_type]
        bad_msgs = product(msgs, types)

        for msg, type_ in bad_msgs:
            with self.assertRaises(TypeError):
                _check_msg_preconditions(msg, type_, label)

        bad_combinations = [(self.pi_tracker_msg, self.nite_skel_msg_type),
                            (self.nite_skel_msg, self.pi_tracker_msg_type)]
        
        for msg, type_ in bad_combinations:
            with self.assertRaises(TypeError):
                _check_msg_preconditions(msg, type_, label)


    def test_check_preconditions_raises_with_bad_labels(self):
        bad_labels = ['UNKNOWN', None, [], 123]
        for bl in bad_labels:
            with self.assertRaises(TypeError):
                _check_msg_preconditions(self.pi_tracker_msg, 
                                         self.pi_tracker_msg_type, 
                                         bl)

    def test_check_preconditions(self):
        label = 'lalala'
        try:
            _check_msg_preconditions(self.pi_tracker_msg, Skeleton, label)
            _check_msg_preconditions(self.nite_skel_msg, NiteSkeletonList, label)
        except:
            self.fail()


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
        bad_msgs = ((None, 'lalala'),  (123, 'lalala'), 
                    (self.skel, None), (self.skel, 123), (self.skel, 'UNKNOWN'), 
                    (self.skel, None), (self.skel, []))
        for instance, label in bad_msgs:
            with self.assertRaises(TypeError):
                self.builder.parse_msg(instance, label)
        
    def test_parse_msg(self):
        self.assertEqual(self.instance, 
                         self.builder.parse_msg(self.skel, 'lalala'))


    def test_get_msg_class(self):
        skel = self.builder.get_msg_class()()
        self.assertTrue(isinstance(skel, Skeleton))


class TestKinectIBuilder(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestKinectIBuilder, self).__init__(*args)

    def setUp(self):
        self.builder = KinectIBuilder()
        self.skel_list = NiteSkeletonList()
        self.skel = NiteSkeleton()

    def make_instance(self):
        return PoseInstance(colums=self.builder.columns,
                            label='lalala',
                            instance=nsku.unpack_skeleton_msg(self.skel))   
    def tearDown(self):
        pass

    def test_check_parse_msg_preconditions_raises_with_empty_msg(self):
        with self.assertRaises(TypeError):
            self.builder.check_parse_msg_preconditions(self.skel_list, 'lalala')

    def test_check_parse_msg_preconditions(self):
        self.skel_list.skeletons = [NiteSkeleton(), NiteSkeleton()]
        try:
            self.builder.check_parse_msg_preconditions(self.skel_list, 'lalala')
        except:
            self.fail()




if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_check_msg_preconditions', TestPreconditions)
    rosunit.unitrun(PKG, 'test_PiTrackerIBuiler', TestPiTrackerIBuiler)
    rosunit.unitrun(PKG, 'test_KinectIBuilder', TestKinectIBuilder)
