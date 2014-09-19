#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_labeler')
import rospy
import asr_msgs.msg as asrmsg
from std_msgs.msg import String

# pose_labels = ('TURNED_RIGHT', 'TURNED_LEFT', 'TURNED_FORWARD',
#     		'LOOKING_RIGHT', 'LOOKING_LEFT', 'LOOKING_FORWARD',
#     		'POINTING_RIGHT', 'POINTING_LEFT', 'POINTING_FORWARD',
#     		'TURNED_RIGHT', 'TURNED_LEFT', 'TURNED_FORWARD',
#     		'LOOKING_RIGHT', 'LOOKING_LEFT', 'LOOKING_FORWARD',
#     		'POINTING_RIGHT', 'POINTING_LEFT', 'POINTING_FORWARD')

# commands = ('START', 'CHANGE', 'PAUSE', 'CONTINUE', 'END')


class PoseLabeler():
    def __init__(self):
        rospy.init_node('pose_labeler')
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing " + self.node_name + " node...")

        # Subscribe to the asr_recognition_results topic.
        rospy.Subscriber("recognition_results", asrmsg.recog_results,
                        self.results_callback)
        # Publishers
        self.label_pub = rospy.Publisher('pose_label', String)
        self.command_pub = rospy.Publisher('pose_command', String)

        try:
            # Parameters defining valid poses and commands 
            # to control the training
            pose_labels = rospy.get_param('pose_labels')
            commands = rospy.get_param('pose_commands')
            # self.min_confidence = rospy.get_param(rospy.get_name() + 'min_confidence', 0.5)
            self.min_confidence = rospy.get_param('~min_confidence', 0.5)
        except KeyError:
            rospy.logfatal("PoseLabeler: Error when loading parameters!")
            rospy.signal_shutdown("node " + rospy.get_name() \
                + " shot down because parameters were not found")

        # Associate labels and commandsto their own publisher
        label_pairs = [(l, self.label_pub) for l in pose_labels]
        command_pairs = [(c, self.command_pub) for c in commands]
        self.label_publishers = dict(label_pairs + command_pairs)
        
        rospy.spin()

    def results_callback(self, results):
        if not results:
            rospy.logwarn("Received empty message.")
            return
        if not results.semantics:
            rospy.logwarn("Received message without semantics.")
            return

    	label = '_'.join([sem.value for sem in results.semantics])
    	# Average confidence of the recognition:
    	confidence = sum([sem.confidence for sem in results.semantics]) / len(results.semantics) # todo should catch division by 0
    	if confidence < self.min_confidence:
            rospy.logwarn(  "Confidence is too low for label: %s (%f)", 
                            label, confidence)
            return
        
        if label not in self.label_publishers:
            for pub in set(self.label_publishers.values()):
                pub.publish('UNKNOWN')  # Publish 'UNKNOWN' in all publishers
            rospy.logwarn(rospy.get_name() + " UNKNOWN label: %s", label)
            return

        # Publish the label to its corresponding publisher:
        self.label_publishers[label].publish(label) 
        rospy.loginfo("(" + rospy.get_name() + ") Label: %s", label)

        
    def shutdown(self):
        rospy.loginfo('Shutting down ' + rospy.get_name() + ' node.')

    
if __name__ == '__main__':
    try:
        PoseLabeler()
    except rospy.ROSInterruptException:
        pass
