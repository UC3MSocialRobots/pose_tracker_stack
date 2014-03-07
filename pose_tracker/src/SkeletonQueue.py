import roslib; roslib.load_manifest('pose_tracker')
import rospy

from std_msgs.msg import String
import kinect.msg as kin

import collections as col
import pandas as pd
import itertools as it

import kinect.nite_msg_utils.nite_skeleton_utils as nsku

class SkeletonQueue(object):
    """docstring for SkeletonQueue"""
    def __init__(self, joint_names, *args):
        super(SkeletonQueue, self).__init__()
        self.skeleton_queue = col.deque([]) # to store (skeletons, label)
        self.joint_names = joint_names

    def __len__(self):
        return len(self.skeleton_queue)

    def append(self, skeletons, label):
        ''' appends a skeleton and a label to the queue '''
        self.skeleton_queue.append((skeletons, label))

    def clear(self):
        ''' removes all elements of the queue'''
        self.skeleton_queue.clear()
    
    def _check_joints(self, joint_names1, joint_names2):
        ''' Raises TypeError if entered parameters differ'''
        if set(joint_names1) != set(joint_names2):
            rospy.logwarn("Message joint name does not match to the expected one")
            raise TypeError(
                "Message does not have the expected joint names\n"
                "Received: {}\nExpected: {}"
                .format(str(joint_names1), str(joint_names2)))

    def _process_skeleton_msg(self, skeleton):
        ''' Unpacks a skeleton message frome the queue, 
            checks if joints are valid and returns its data '''
        try:
            data, joint_names = nsku.unpack_skeleton_msg(skeleton)
            self._check_joints(joint_names, self.joint_names)
            return data
        except TypeError as e:
            raise

    def _calc_chunksize(self, chunksize):
        ''' Calculates how many elements to retrieve from squeleton queue'''
        if chunksize < 0 or chunksize > len(self.skeleton_queue):
            return len(self.skeleton_queue)
        return chunksize 

    def _pop_from_queue(self, elements):
        ''' Generator that pops n elements from skeleton_queue.
            NOTE: this is a generator, meaning that it executes LAZYLY!
            NOTE: It only grabs the first skeleton from the NiteSkeletonList msg
        '''
        for _ in xrange(self._calc_chunksize(elements)):
            try:
                skels, label = self.skeleton_queue.popleft()
                skel_data = self._process_skeleton_msg(skels.skeletons[0])
                yield it.chain(skel_data, label)   
            except TypeError, e:
                rospy.logwarn("Message not added to the dataset\n"
                              "Reason: {}".format(e))
            except IndexError, e:
                rospy.logdebug("Skeleton Queue got emptied. Stoping")               
                break

    def _prepare_chunk(self, chunksize=50, **kwargs):
        ''' Yields chunksize elements from self.skeleton_queue (using popleft())
            Arguments:
            :chunksize=50: Num of elements to retrieve from the skeleton_queue.
                           If chunksize < 0  or > len self.skeleton_queue,
                           then yelds all elements of the queue
        '''
        if not self.skeleton_queue:
            rospy.logdebug("The skeleton skeleton_queue is empty. Nothing to do.")
            return

        for skel_data in self._pop_from_queue(chunksize):
            yield skel_data
        
    def _chunk_to_data_frame(self, chunk, columns):
        ''' Returns a pandas.DataFrame of chunksize elements.
            Note that this method expects its input from'''
        return pd.DataFrame(list(chunk), columns=columns)

    def pop_n_to_DataFrame(self, n, columns):
        ''' Pops n elements from the queue and returns them as a pandas.DataFrame'''
        chunk = self._prepare_chunk(n)
        return self._chunk_to_data_frame(chunk, columns)
