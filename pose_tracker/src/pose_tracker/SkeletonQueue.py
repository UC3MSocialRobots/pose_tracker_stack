import roslib; roslib.load_manifest('pose_tracker')
import rospy

import collections as col
import pandas as pd
from itertools import chain

import kinect.nite_skeleton_msg_utils as nsku


class SkeletonQueue(object):

    """
    Class that contains a queue of skeletons
    along with operations return its eleements as a pandas.DataFrame
    """

    def __init__(self, joint_names):
        super(SkeletonQueue, self).__init__()
        self.skeleton_queue = col.deque([])  # stores (skeletons, label)
        self.joint_names = joint_names

    def __len__(self):
        return len(self.skeleton_queue)

    def append(self, skeletons, label):
        """Append a skeleton and a label to the queue."""
        self.skeleton_queue.append((skeletons, label))

    def clear(self):
        """Remove all elements of the queue."""
        self.skeleton_queue.clear()

    def _check_joints(self, joint_names1, joint_names2):
        """Raise TypeError if entered parameters differ."""
        if set(joint_names1) != set(joint_names2):
            rospy.logwarn("Message joint name != expected one")
            raise TypeError(
                "Message does not have the expected joint names\n"
                "Received: {}\nExpected: {}"
                .format(str(joint_names1), str(joint_names2)))

    def _process_skeleton_msg(self, skeleton):
        """
        Return unpacked data from a skeleton if its joints are valid..

        Unpack a skeleton message frome the queue,
        check if joints are valid and returns its data.
        """
        try:
            data, joint_names = nsku.unpack_skeleton_msg(skeleton)
            self._check_joints(joint_names, self.joint_names)
            return data
        except TypeError as e:
            raise e

    def _calc_chunksize(self, chunksize):
        """Calculate how many elements to retrieve from squeleton queue."""
        if chunksize < 0 or chunksize > len(self.skeleton_queue):
            return len(self.skeleton_queue)
        return chunksize

    def _pop_from_queue(self, elements):
        """
        Pop n elements from skeleton_queue.

        @note: this is a generator, meaning that it executes LAZYLY!
        @note: only grabs the first skeleton from the NiteSkeletonList msg
        """
        for _ in xrange(self._calc_chunksize(elements)):
            try:
                skels, label = self.skeleton_queue.popleft()
                skel_data = self._process_skeleton_msg(skels.skeletons[0])
                yield chain(skel_data, label)
            except TypeError, e:
                rospy.logwarn("Message not added to the dataset\n"
                              "Reason: {}".format(e))
            except IndexError, e:
                rospy.logdebug("Skeleton Queue got emptied. Stoping")
                break

    def _prepare_chunk(self, chunksize=50):
        """
        Yield chunksize elements from self.skeleton_queue (using popleft()).

        @param chunksize: Num of elements to retrieve from the queue.
        @note: If chunksize < 0  or > len self.skeleton_queue,
            then yelds all elements of the queue
        """
        if not self.skeleton_queue:
            rospy.logdebug("Skeleton_queue is empty. Nothing to do.")
            return

        for skel_data in self._pop_from_queue(chunksize):
            yield skel_data

    def _chunk_to_data_frame(self, chunk, columns):
        """
        Return a pandas.DataFrame of chunksize elements.

        @note: This method expects its input from L{_prepare_chunk}
        @param chunk: a chunk of elements obtained from L{_prepare_chunk}
        @param columns: list containing the names of the
            pandas.DataFrame columns
        @return: the elements of queue in form of a pandas.DataFrame
        @rtype: pandas.Dataframe
        """
        return pd.DataFrame(list(chunk), columns=columns)

    def pop_n_to_DataFrame(self, n, columns):
        """
        Pop n elements from queue and returns them as a L{pandas.DataFrame}.

        @param n: number of elements to retrieve from queue
        @param columns: list with the names of the pandas.DataFrame columns
        @return: n elements from the queue in form of a pandas.DataFrame
        @rtype: pandas.DataFrame
        """
        chunk = self._prepare_chunk(n)
        return self._chunk_to_data_frame(chunk, columns)
