#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_tracker')
import rospy

from pose_tracker.srv import State
from std_msgs.msg import String
import kinect.msg as kin

import datetime
import pandas as pd
import itertools as it
from functools import wraps
# import os

import param_utils as pu
import PoseDatasetIO as pdio
import SkeletonQueue as skq


DEFAULT_NAME = 'pose_dataset_builder'

PARAM_NAMES = ('dataset', 'rate', 'pose_labels', 'pose_commands', 
               'command_mapper', 'skeleton_joints', 'joint_attrib_names')

STATE_INIT = 'initiating'
STATE_IDLE = 'idle'
STATE_PROCESSING = 'processing'
STATE_FINISHING = 'finishing'
STATE_END = 'end'
ALL_STATES = ( STATE_INIT, STATE_IDLE, STATE_PROCESSING, 
               STATE_FINISHING, STATE_END)

def call_only_in(states):
    '''Decorator method that ensures that decorated method is
       only called in the entered states.

       @param states: 'A list of the states in which the decorated method will be called'
              
       Example: 

            >>> @_call_only_in([STATE_PROCESSING,])
            ... def f1(self): 
            ...     print "f1 called!" 
            >>> builder = PoseDatasetBuilder()
            >>> builder.curr_state = STATE_IDLE
            >>> f1() # f1 does nothing
            self.curr_state = STATE_PROCESSING
            >>> f1() 
            'f1 called!'
    '''
    # Helper func to convert an object to an iterable (unless already it is one)
    as_iter = lambda i: i if hasattr(i, '__iter__') else (i,)
    states = set(as_iter(states))
    
    if not states.issubset(set(ALL_STATES)):
        raise KeyError('States "{}" not in "{}"'.format(states, ALL_STATES))
    def method_wrapper(method):
        @wraps(method)
        def caller(self, *args, **kwargs):
            if self.curr_state in states:
                return method(self,*args, **kwargs)
            rospy.logwarn("'{}'' cannot be called in state '{}'"
                .format(method.__name__, self.curr_state))
        return caller
    return method_wrapper


class PoseDatasetBuilder():
    ''' Class that builds a dataset
        @keyword nodename: The name of the node
    '''
    def __init__(self, **kwargs):
        name = kwargs.get('node_name', DEFAULT_NAME)
        rospy.init_node(name)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing " + self.node_name + " node...")

        self.current_label = None
        self.all_labels = set()     # A set to keep track of the used labels

        # Dict mapping state names to the functions executed in those states
        self.states = { STATE_INIT:         self.state_initiating,
                        STATE_IDLE:         self.state_idle, 
                        STATE_PROCESSING:   self.state_processing, 
                        STATE_FINISHING:    self.state_finishing,
                        STATE_END:          self.state_end }
        self.curr_state = STATE_IDLE

        self.transitions = {
            STATE_INIT:         (STATE_IDLE),
            STATE_IDLE:         (STATE_IDLE, STATE_PROCESSING, STATE_FINISHING), 
            STATE_PROCESSING:   (STATE_IDLE, STATE_PROCESSING, STATE_FINISHING), 
            STATE_FINISHING:    (STATE_END),
            STATE_END:          (STATE_END) }

        # Subscribers
        rospy.Subscriber("pose_command", String, self.command_callback)
        rospy.Subscriber("pose_label", String, self.label_callback)
        rospy.Subscriber("skeletons", kin.NiteSkeletonList, self.skeleton_callback)

        # Publishers
        self.ready_pub = rospy.Publisher('~dataset_ready', String)
        self.state_pub = rospy.Publisher("~state_changed", String)

        # Services
        self.state_srv = rospy.Service('~state', State, self.handle_state_srv)

        # Parameter Loading
        self.load_parameters()
        
        # Combine joint names with joint attributes 
        # to create the dataset's attribute names (columns)
        self.dataset_columns = self._make_dataset_columns()
            
        self.dataset_columns.insert(0, 'time_stamp')
        self.dataset_columns.insert(0, 'user_id')
        self.dataset_columns.append('label')

        # Stores (skeletons, label) pairs and manages queue IO
        self.skeleton_queue = skq.SkeletonQueue(self.joint_names) 

        # # Init State Machine
        # self.state_machine()
        # rospy.spin()

    def _make_dataset_columns(self):
        ''' Helper method that returs a list with dataset columns.
        @note:: PoseDatasetBuilder.joint_names and PoseDatasetBuilder.attrib_names 
                must be declared prior to call this method '''
        return map('_'.join, it.product(self.joint_names, self.attrib_names))

    def load_parameters(self):
        ''' Loads all the parameters from the ros master'''
        all_params = pu.get_parameters(PARAM_NAMES)
        try:
            index = 0
            dataset_config = all_params.next().value
            self.dataset_name = dataset_config['filename']
            rospy.loginfo("Using dataset: " + str(self.dataset_name))

            self.dataset_metadata = dataset_config['metadata']
            rospy.loginfo("Dataset metadata" + str(self.dataset_metadata))

            self.table_name = dataset_config['table_name']
            rospy.loginfo("Dataset data table: " + str(self.table_name))

            self.append_data = dataset_config['append_to_table']
            rospy.loginfo("Append to table: " + str(self.append_data))
            
            index += 1
            self.rate_param = all_params.next().value
            self.rate = rospy.Rate(self.rate_param) 
            rospy.loginfo("Processing data at %dHz", self.rate_param)
            
            index += 1
            self.pose_labels = all_params.next().value
            rospy.loginfo("Pose labels: " + str(self.pose_labels))
            
            index += 1
            self.pose_commands = all_params.next().value
            rospy.loginfo("Pose commands: " + str(self.pose_commands))
            
            index += 1
            self.command_mapper = all_params.next().value
            rospy.loginfo("Command mapper: " + str(self.command_mapper))

            index += 1
            self.joint_names = all_params.next().value
            rospy.loginfo("Joint names: " + str(self.joint_names))
            
            index += 1
            self.attrib_names = all_params.next().value
            rospy.loginfo("Attrib names" + str(self.attrib_names))

        except KeyError:
            rospy.logfatal("Error when loading parameter '{}'".
                format(PARAM_NAMES[index]))
            rospy.signal_shutdown("node " + rospy.get_name() + \
                " shot down because parameters were not found")


    def change_state(self, new_state):
        ''' Tries to change the current state. 
            If the state change is not allowed, it does nothing.

            @param new_state: the new state to be set. Should be included in 
                              I{ALL_STATES}
        '''
        if new_state == self.curr_state: # Exit if the sate is the same
            rospy.logdebug("We are already in " + self.curr_state)           
            return

        # Check if we have a valid transition
        if new_state not in self.transitions.get(self.curr_state):
            rospy.logdebug("Invalid state transition. State not changed")
            return

        self.curr_state = new_state
        self.state_pub.publish(self.curr_state)
        rospy.logdebug("State changed to " + self.curr_state)       
        rospy.logwarn("State changed to " + self.curr_state)       


    # --- Topic callbacks ---
    def command_callback(self, command):
        ''' Processes the received command and sets the current state 
            according to this command.
            See self.command_mapper dict to know the command->state mappings
        '''
        if self.curr_state == STATE_END:
            rospy.loginfo("Already in State:END. Too late to change the state")
            return

        if command.data not in self.command_mapper:
            rospy.logwarn("State could not be changed since command '" \
                            + str(command.data) + "' does not map to any state")
            return

        self.change_state(self.command_mapper[command.data])
    
    @call_only_in(STATE_PROCESSING)
    def label_callback(self, label):
        ''' Updates the label of the received data ''' 
        self.current_label = label.data
        rospy.loginfo("Received label:" + self.current_label)
        if label.data != 'UNKNOWN':
            self.all_labels.add(label.data) # Update the set of used labels

    @call_only_in(STATE_PROCESSING)
    def skeleton_callback(self, skeletons):
        ''' Adds the received skeletons message to the queue 
            together with the current label in a form of:
            tuple (skeleton, label)
            The skeleton is added if:
            - State is processing
            - Label is set
            - Label != "UNKNOWN" 
        '''
        if self.curr_state != STATE_PROCESSING:
            return
        if not skeletons.skeletons:      # Skeleton list is empty
            rospy.loginfo("No user found. Skeleton list is empty.")
            return
        if not self.current_label:
            rospy.loginfo("Label is no set. Skeleton msg discarded.")
            return
        if self.current_label == 'UNKNOWN':
            rospy.loginfo('UKNOWN label. Skeleton msg discarded.')
            return
        # self.skeleton_queue.append((skeletons, self.current_label))
        self.skeleton_queue.append(skeletons, self.current_label)

    def handle_state_srv(self):
        ''' Service callback to respond the request asking the current state.'''
        return State.StateResponse(self.curr_state)

    # --- State functions --- 
    def state_initiating(self):
        ''' Initial state.
            Creates dataset file and fills metadata.
            If parameter new_dataset is passed, then 
        '''
        rospy.logdebug('State: Initiating')
        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
        self.dataset_metadata['date'] = pdio.parse_date(now)
        # Preparing dataset file     
        self.data_writer = pdio.PoseDatasetIO(dataset=self.dataset_name, 
                                              columns=self.dataset_columns)
        self.data_writer.create_dataset()
        self.data_writer.fill_metadata(**self.dataset_metadata)
        # End Init. Change to state idle
        self.change_state(STATE_IDLE)
        

    def state_idle(self):
        ''' Performs operations of the sate "idle"'''
        # rospy.loginfo('State: Idle')
        pass


    def state_processing(self):
        ''' Processes skeletons from the queue and adds them to the dataset'''
        rospy.logdebug('State: Processing')

        if not self.all_labels:
            rospy.loginfo('Label is still not set. Nothing to write')
            return
        if not self.skeleton_queue:
            rospy.logdebug('Processing state, but Skeletons Queue is empty')
            return
       # We process one chunk per second
        df = self.skeleton_queue \
            .pop_n_to_DataFrame(self.rate,self.dataset_columns)
        self.data_writer.write(self.table_name, df, 
                                table=True, append=self.append_data)
        # After the first time we write, we append the data
        self.append_data = True
    

    def _write_labels_to_file(self, table_name):
        '''Helper method that writes all labels to dataset  
           params: 
           @name: table_name 
           the name of the table where the labels will be stored'''
        self.data_writer.write(table_name, 
                                    pd.Series(list(self.all_labels)))

    def state_finishing(self):
        ''' Transitionalstate to state_end.
            Dumps remaining skeletons to the dataset and closes the file'''
        rospy.loginfo('State: finishing')
        try:
            # Write to the file the remaining skeletons of the queue
            df = self.skeleton_queue.pop_n_to_DataFrame(-1,self.dataset_columns)
            self.data_writer.write(self.table_name, df,
                                    table=True, append=self.append_data)
            
            # Store a list with all labels if data in dataset
            self._write_labels_to_file('used_labels')
           
            # Close the file
            self.data_writer.store.close()
            self.ready_pub.publish(self.dataset_name)
        except:
            rospy.logdebug("In shutdown: file is already closed")
        self.change_state(STATE_END)
        self.states[self.curr_state]()


    def state_end(self):
        ''' Final state of the state machine. Does nothing '''
        rospy.loginfo('State: END')
        
    def run_current_state(self): 
        ''' Executes the method corresponding to the current state '''
        self.states[self.curr_state]()

    def state_machine(self):
        ''' Executes the state machine'''
        while not rospy.is_shutdown():
            if self.curr_state != STATE_END:
                self.run_current_state()
            self.rate.sleep()


    # def __get_closest_skeleton(self, skeletons, joint="torso"):
    #     ''' Returns the closest skeleton by comparing 
    #         the z position of a list of skeletons
    #         Input: skeletons - The list of skeletons
    #         Input: joint="torso" - The joint which is used to compare skeletons
    #         Return: The index of the closest skeleton
    #     '''
    #     # TODO
    #     pass

    
    def shutdown(self):
        ''' Closes the node ''' 
        if self.curr_state != STATE_END:
            self.states[STATE_FINISHING]()
        try:
            self.state_srv.shutdown()
        except:
            pass
        rospy.loginfo('Shutting down ' + rospy.get_name() + ' node.')


if __name__ == '__main__':
    try:
        node = PoseDatasetBuilder()
        node.state_machine()
    except rospy.ROSInterruptException:
        pass
