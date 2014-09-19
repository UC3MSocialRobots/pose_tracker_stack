cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined servBices
rosbuild_gensrv()

# rosbuild_add_executable(skeleton_frame_transformer src/skeleton_frame_transformer.cpp)

rosbuild_add_pyunit(src/test/pose_tracker/test_PoseDatasetIO_writer.py)
rosbuild_add_pyunit(src/test/pose_tracker/test_PoseDatasetIO_reader.py)
rosbuild_add_pyunit(src/test/pose_tracker/test_SkeletonQueue.py)
rosbuild_add_pyunit(src/test/pose_tracker/test_PoseDatasetIO_dateParser.py)
rosbuild_add_pyunit(src/test/pose_tracker/test_only_in_states.py)
rosbuild_add_rostest(test/pose_dataset_builder.test)

rosbuild_add_pyunit(src/test/pose_detector/test_circular_dataframe.py)
rosbuild_add_pyunit(src/test/pose_detector/test_calc_joint_velocities.py)
rosbuild_add_pyunit(src/test/pose_detector/test_pose_detectors.py)
rosbuild_add_rostest(test/joint_velocities_publisher.test)
rosbuild_add_rostest(test/pose_detector_node.test)
#rosbuild_add_rostest(test/pose_detector_node_changes_detectors.test)