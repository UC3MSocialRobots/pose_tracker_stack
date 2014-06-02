#!/usr/bin/env python
PKG = 'pose_tracker'
import roslib
roslib.load_manifest(PKG)

import unittest
from mock import patch
from itertools import chain

import kinect.msg as kin
import kinect.nite_skeleton_msg_utils as nsku
import pose_tracker.SkeletonQueue as skq


class TestSkeletonQueue(unittest.TestCase):
    """docstring for TestSkeletonQueue"""
    def __init__(self, *args):
        super(TestSkeletonQueue, self).__init__(*args)
        self.joint_names = ['head', 'neck']
        self.skq = skq.SkeletonQueue(self.joint_names)

    def setUp(self):
        pass

    def tearDown(self):
        self.skq.skeleton_queue.clear()

    # @unittest.skip("Skpping this Test")
    def test_process_skeleton_msg(self):
        good_skel = nsku.generate_fake_skelmsg(self.joint_names)
        unpacked = nsku.unpack_skeleton_msg(good_skel)
        unpacked = list(unpacked[0])
        processed = list(self.skq._process_skeleton_msg(good_skel))
        self.assertEqual(processed, unpacked,
                         "_process_skeleton_msg did not returned "
                         "what expected.\nExpected: {}\nGot: {}".
                         format(unpacked, processed))

    def test_process_skeleton_msg_raises_TypeError_if_no_skeleton_passed(self):
        self.assertRaises(TypeError, self.skq._process_skeleton_msg, None)

    def test_process_skeleton_msg_raises_TypeError_if_receives_empty_skel(self):
        skel = kin.NiteSkeleton()
        self.assertRaises(TypeError, self.skq._process_skeleton_msg, skel)

    def test_process_skeleton_msg_raises_TypeError_if_joints_not_valid(self):
        bad_skel = nsku.generate_fake_skelmsg(['hhhead', 'ne33ck'])
        self.assertRaises(TypeError, self.skq._process_skeleton_msg, bad_skel)

    def _fill_skel_queue(self, nelements, joint_names,
                         skels_per_msg=2, label='label'):
        ''' Helper function that ads n nelements to the skeleton queue.
            parameters:
            @name nelements Num of NiteSkeletonList messages to add to the queue
            @type int
            @name label label that is added to each message
            @type string
            @name joint_names joint names of each skeleton message
            @type list of strings
            @name skels_per_msg=2 num of skeletons per each NiteSkeletonList msg
            @type int
        '''
        msg_list = []
        for i in xrange(nelements):
            fake_skels = nsku.generate_fake_NiteSkeletonList_msg(skels_per_msg,
                                                                 joint_names)
            q_item = [fake_skels, label]
            self.skq.skeleton_queue.append(q_item)
            msg_list.append(q_item)
        return msg_list

    def test_pop_from_queue_pops_correct_num_of_elements(self):
        # self.joint_names = ['head', 'neck']
        label = 'label_1'
        # self.skq.all_labels.add(label)
        self._fill_skel_queue(4, self.joint_names, label=label)

        list(self.skq._pop_from_queue(0))
        q_length = len(self.skq.skeleton_queue)
        self.assertEqual(4, q_length, "Queue should have remained untouched")

        popped = list(self.skq._pop_from_queue(2))
        q_length = len(self.skq.skeleton_queue)
        self.assertEqual(2, q_length, "Queue should have now only 2 elements! "
                                      "It has {} elements".format(q_length))

        # retrieving more elements that already are in queue
        popped = list(self.skq._pop_from_queue(4))
        q_length = len(self.skq.skeleton_queue)
        self.assertEqual(q_length, 0, "Queue should have been emptied")
        self.assertEqual(len(popped), 2,
                         "Should have retrieved the last {} elems from queue. "
                         "Retrieved: {}\nPopped elements: {}"
                         .format(2, len(popped), popped))

    def test_pop_from_queue_does_not_return_messages_with_invalid_joints(self):
        # self.joint_names = ['head', 'neck']
        invalid_joint_names = ['aaaa', 'bbbb']

        self._fill_skel_queue(4, invalid_joint_names, label='label')
        popped = list(self.skq._pop_from_queue(4))
        q_length = len(self.skq.skeleton_queue)
        self.assertEqual(q_length, 0, "Queue should have been emptied. "
                                      "There still are {} elements in queue."
                                      .format(q_length))
        self.assertEqual(len(popped), 0, "Shouldn't have returned any skeleton."
                                         "Num of returned skeletons {}."
                                         .format(len(popped)))

    def test_pop_from_queue_returns_a_skeleton_and_a_label(self):
        # self.joint_names = ['head', 'neck']
        label = 'label_1'
        # self.skq.all_labels.add(label)
        msg_list = self._fill_skel_queue(4, self.joint_names, label=label)

        popped = list(self.skq._pop_from_queue(4))
        for i, item in enumerate(popped):
            item = list(item)
            message_skel = msg_list[i][0]
            message_label = msg_list[i][1]
            unpacked_skel = \
                nsku.unpack_skeleton_msg(message_skel.skeletons[0])[0]
            target_to_compare = list(chain(unpacked_skel, message_label))
            self.assertEqual(item, target_to_compare,
                             "Messages are not equal: {} != {}"
                             .format(item, target_to_compare))

    def test_calc_chunksize(self):
        self.skq.skeleton_queue.clear()
        self.skq.skeleton_queue.extend(xrange(10))
        self.assertEqual(self.skq._calc_chunksize(8), 8)
        for i in [-1, 12]:
            self.assertEqual(self.skq._calc_chunksize(i),
                             len(self.skq.skeleton_queue))

    # @unittest.skip("Skpping this Test")
    @patch.object(skq.SkeletonQueue, '_pop_from_queue')
    def test_prepare_chunk_returns_None_when_queue_is_empty(self, mock_pop):
        self.skq.skeleton_queue.clear()
        self.assertEqual(list(self.skq._prepare_chunk()), [])

    # @unittest.skip("Skpping this Test")
    @patch.object(skq.SkeletonQueue, '_pop_from_queue')
    def test_prepare_chunk(self, mock_pop):
        mock_pop.side_effect = lambda x: xrange(x)

        self.skq.skeleton_queue.clear()
        self.assertEqual(list(self.skq._prepare_chunk()), [],
                         "Should return 'None' when queue is empty")

        # Testing
        self.skq.skeleton_queue.clear()
        self.skq.skeleton_queue.extend(xrange(10))
        list(self.skq._prepare_chunk(8))
        mock_pop.assert_called_with(8)

    def test_to_dataframe(self):
        '''ToDo'''
        pass


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_skeleton_queue_IO', TestSkeletonQueue)
