#!/usr/bin/env python
PKG = 'pose_tracker'
NNAME = 'test_PoseDatasetBuilder'
import roslib; roslib.load_manifest(PKG)
import rospy
import unittest
from mock import patch, MagicMock

import pose_tracker.pose_dataset_builder_node as pdb
import param_utils as pu

class TestPoseDatasetBuilderInit(unittest.TestCase):
    """Tests initialization of the node"""
    def __init__(self, *args):
        super(TestPoseDatasetBuilderInit, self).__init__(*args)
            
    def setUp(self):
        # Mock IO to speedup tests
        pdb.rospy.Subscriber = MagicMock()
        pdb.rospy.Service = MagicMock()
        pdb.pdio = MagicMock()
        pass
        

    def tearDown(self):
        pass

    @patch('pose_tracker.PoseDatasetIO.PoseDatasetIO')
    def test_init_ok(self, mock_pdio):
        try:
            pdb.PoseDatasetBuilder()
        except Exception, e:
            self.fail("PoseDatasetBuilder should have not died!\n"
                      "Reason: " + str(e.message))

    @patch.object(pu, 'get_parameters')
    def test_die_if_param_not_found(self, mock_putils):
        node = None
        with patch.object(rospy, 'signal_shutdown') as mock_shutdown:
            mock_putils.side_efect = pu.ParamNotFoundError()
            node = pdb.PoseDatasetBuilder()
            mock_shutdown.assert_called()
        # node.state_srv.shutdown('Manually shutting down the service ')
        # node.shutdown()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_PoseDatasetBuilder_init', 
                    TestPoseDatasetBuilderInit)
