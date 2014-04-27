#!/usr/bin/env python
import roslib; roslib.load_manifest('pose_tracker')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from pose_tracker.srv import State
from std_msgs.msg import String
import kinect.msg as kin

import datetime
import pandas as pd
from itertools import product
from functools import wraps
# from contextlib import contextmanager

import param_utils as pu
from func_utils import (error_handler, preconditions, PreconditionError)
from iter_utils import as_iter
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

# class PreconditionError(Exception):
#     ''' Exception that shuold be raised when the preconditions of a function 
#         are not met '''
#     pass

# @contextmanager
# def error_handler(logger=loginfo,  log_msg='',
#                   action=lambda:None, action_args=[], action_kwargs={}):
#     ''' Context Manager that logs errors and takes action
#         @param logger: logging function. Default: loginfo
#         @type logger: callable
#         @param log_msg: message to add to the logger in case of fail 
#                         (It will be preced to the exception message)
#         @type log_msg: str
#         @param action: function to perform if an exception occurs. Default: None
#         @type action: callable
#         @param action_args: argument list to pass to action. Default:[]
#         @param action_kwargs: argument keywords to pass to action. Default: {}'''
#     try:
#         yield
#     except Exception, e:
#         logger(''.join([log_msg, e.message]))
#         if action:
#             action(*action_args, **action_kwargs)

def only_in_states(states):
    '''Decorator method that ensures that decorated method is
       only called in the entered states.

       @param states: 'A list of the states in which the decorated method will be called'
              
       Example: 

            >>> class MyClass():
                    def __init__():
                        self.curr_state = 'idle'
            >>>     @only_in_states(['state_1',])
            ...     def f1(self): 
            ...         print "f1 called!" 
            >>> klass = MyClass()
            >>> klass.curr_state = 'idle'
            >>> klass.f1() # f1 does nothing
            >>> klass.curr_state = 'state_1'
            >>> klass.f1() 
            ... 'f1 called!'
    '''
    # Helper func to convert an object to an iterable (unless already it is one)
    states = set(as_iter(states))
    
    if not states.issubset(set(ALL_STATES)):
        raise KeyError('States "{}" not in "{}"'.format(states, ALL_STATES))
    def method_wrapper(method):
        @wraps(method)
        def caller(self, *args, **kwargs):
            if self.curr_state in states:
                return method(self,*args, **kwargs)
            logwarn("'{}'' cannot be called in state '{}'"
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
        loginfo("Initializing " + self.node_name + " node...")

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

        # Parameter Loading. Shutdown node if failure occurs.
        with error_handler(logger=logfatal, action=self.shutdown):
            self.load_parameters()
        
        # Combine joint names with joint attributes 
        # to create the dataset's attribute names (columns)
        self.dataset_columns = self._combine_joints_attribs(self.joint_names, 
                                                          self.attrib_names)
            
        self.dataset_columns.insert(0, 'time_stamp')
        self.dataset_columns.insert(0, 'user_id')
        self.dataset_columns.append('pose')

        self.dataset_config['columns'] = self.dataset_columns
        rospy.set_param('~dataset', self.dataset_config)
        #   rospy.set_param('~dataset/columns', self.dataset_columns)

        # Stores (skeletons, label) pairs and manages queue IO
        self.skeleton_queue = skq.SkeletonQueue(self.joint_names) 

    def _combine_joints_attribs(self, joint_names, attrib_names):
        ''' Helper method that returs a list with dataset columns.
        @note:: PoseDatasetBuilder.joint_names and PoseDatasetBuilder.attrib_names 
                must be declared prior to call this method '''
        return map('_'.join, product(joint_names, attrib_names))

    def load_parameters(self):
        ''' Loads all the parameters from the ros master'''
        all_params = pu.get_parameters(PARAM_NAMES)
        logger = logwarn

        try:
            self.dataset_config = all_params.next().value
            self.dataset_name = self.dataset_config['filename']
            logger("Using dataset: " + str(self.dataset_name))

            self.dataset_metadata = self.dataset_config['metadata']
            logger("Dataset metadata" + str(self.dataset_metadata))

            self.table_name = self.dataset_config['table_name']
            logger("Dataset data table: " + str(self.table_name))

            self.append_data = self.dataset_config['append_to_table']
            logger("Append to table: " + str(self.append_data))
            
            self.rate_param = all_params.next().value
            self.rate = rospy.Rate(self.rate_param) 
            logger("Processing data at %dHz", self.rate_param)
            
            self.pose_labels = all_params.next().value
            logger("Pose labels: " + str(self.pose_labels))
            
            self.pose_commands = all_params.next().value
            logger("Pose commands: " + str(self.pose_commands))
            
            self.command_mapper = all_params.next().value
            logger("Command mapper: " + str(self.command_mapper))

            self.joint_names = all_params.next().value
            logger("Joint names: " + str(self.joint_names))
            
            self.attrib_names = all_params.next().value
            logger("Attrib names" + str(self.attrib_names))

        except Exception, e:
            logfatal(e.message + " Error when loading parameters: {}".
                format(list(all_params)))
            rospy.signal_shutdown("node " + rospy.get_name() + \
                " shot down because parameters were not found")

    # --- Topic callbacks ---

    def _command_cb_preconditions(self, command):
        if self.curr_state == STATE_END:
            raise PreconditionError("Already in State:END. \
                                     Too late to change the state")
        if command.data not in self.command_mapper:
            raise PreconditionError(
                "State could not be changed since command '" \
                + str(command.data) + "' does not map to any state")

    @preconditions(_command_cb_preconditions, logger=loginfo, reraise=False)
    def command_callback(self, command):
        ''' Processes the received command and sets the current state 
            according to this command.
            See self.command_mapper dict to know the command->state mappings
        '''
        self.change_state(self.command_mapper[command.data])
        
    @only_in_states(STATE_PROCESSING)
    def label_callback(self, label):
        ''' Updates the label of the received data ''' 
        self.current_label = label.data
        loginfo("Received label:" + self.current_label)
        if label.data != 'UNKNOWN':
            self.all_labels.add(label.data) # Update the set of used labels

    def _skeleton_cb_preconditions(self, skeletons):
        if self.curr_state != STATE_PROCESSING:
            raise PreconditionError("Current state is not {}".format(STATE_PROCESSING))
        if not skeletons.skeletons:      # Skeleton list is empty
            raise PreconditionError("No user found. Skeleton list is empty.")
        if not self.current_label:
            raise PreconditionError("Label is no set. Skeleton msg discarded.")
        if self.current_label == 'UNKNOWN':
            raise PreconditionError('UKNOWN label. Skeleton msg discarded.')
            
    @preconditions(_skeleton_cb_preconditions, logger=loginfo, reraise=False)
    @only_in_states(STATE_PROCESSING)
    def skeleton_callback(self, skeletons):
        ''' Adds the received skeletons message to the queue
            together with the current label in a form of:
            tuple (skeleton, label)
            
            The skeleton is added if:
              - State is processing
              - Label is set
              - Label != "UNKNOWN" 
            @param skeletons: The skeletons message to be added to the queue
            @type skeletons: kinect.msg.NiteSkeletonList
        '''
        self.skeleton_queue.append(skeletons, self.current_label)

    def handle_state_srv(self):
        ''' Service callback to respond the request asking the current state.'''
        return State.StateResponse(self.curr_state)

    def create_dataset(self):
        '''Creates dataset file and fills metadata.'''
        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
        self.dataset_metadata['date'] = pdio.parse_date(now)
        # Preparing dataset file     
        self.data_writer = pdio.PoseDatasetIO(dataset=self.dataset_name, 
                                              columns=self.dataset_columns)
        self.data_writer.create_dataset()
        self.data_writer.fill_metadata(**self.dataset_metadata)


    # --- State functions --- 
    def state_initiating(self):
        ''' Initial state. 
            Transitional state that creates dataset and changes state to IDLE'''
        logdebug('State: Initiating')
        self.create_dataset()
        self.change_state(STATE_IDLE)
        
    def state_idle(self):
        ''' Performs operations of the sate "idle"'''
        # loginfo('State: Idle')
        pass

    def _state_processing_precons(self):
        if not self.all_labels:
            raise PreconditionError('Label is still not set. Nothing to write')
        if not self.skeleton_queue:
            raise PreconditionError('Processing state, but Skeletons Queue is empty')
    
    @preconditions(_state_processing_precons, logger=logdebug, reraise=False)
    def state_processing(self):
        ''' Processes skeletons from the queue and adds them to the dataset'''
        logdebug('State: Processing')
        # We process one chunk per second
        self._write_from_queue(self.rate, self.table_name, self.append_data)
        # After the first time we write, we append the data
        self.append_data = True

    def _write_from_queue(self, items, table_name, append, **kwargs):
        df = self.skeleton_queue.pop_n_to_DataFrame(items, self.dataset_columns)
        self.data_writer.write(table_name, df, table=True, append=append)

    def _write_labels_to_file(self, table_name):
        '''Helper method that writes all labels to dataset  
           
           @param table_name: the name of the table 
                              where the labels will be stored'''
        self.data_writer.write(table_name, pd.Series(list(self.all_labels)))

    def state_finishing(self):
        ''' Transitionalstate to state_end.
            Dumps remaining skeletons to the dataset and closes the file'''
        loginfo('State: finishing')
        try:
            # Write to the file the remaining skeletons of the queue
            self._write_from_queue(-1, self.table_name, True)
            self._write_labels_to_file('used_labels')
            self.data_writer.close()  # Close the file
            self.ready_pub.publish(self.dataset_name)
        except:
            logdebug("Finishing. File is already closed")
        self.change_state(STATE_END)
        self.run_state(STATE_END)


    def state_end(self):
        ''' Final state of the state machine. Does nothing '''
        loginfo('State: END')

    def _change_state_preconditions(self, new_state):
        if new_state == self.curr_state: # Exit if the sate is the same
            raise PreconditionError("We are already in " + self.curr_state)           
        # Check if we have a valid transition
        if new_state not in self.transitions.get(self.curr_state):
            raise PreconditionError("Invalid state transition. State not changed")
    
    @preconditions(_change_state_preconditions, logger=logdebug, reraise=False)   
    def change_state(self, new_state):
        ''' Tries to change the current state. 
            If the state change is not allowed, it does nothing.

            @param new_state: the new state to be set. Should be included in 
                              I{ALL_STATES}
            @return: None if change cannot be done or current state if succeeds
        '''
        self.curr_state = new_state
        self.state_pub.publish(self.curr_state)
        logdebug("State changed to " + self.curr_state)       
        return self.curr_state          

    def run_state(self, state):
        ''' Changes the state and runs it.
            @param state: is the state to run. It must be in L{ALL_STATES} '''
        try:
            self.states.get(self.change_state(state), None)()
        except:
            pass
        
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
        loginfo('Shutting down ' + rospy.get_name() + ' node.')
        

if __name__ == '__main__':
    try:
        node = PoseDatasetBuilder()
        node.state_machine()
    except rospy.ROSInterruptException:
        pass
