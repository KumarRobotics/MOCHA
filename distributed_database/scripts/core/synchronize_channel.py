#!/usr/bin/env python3

import enum
import threading
import smach
import synchronize_utils as su
import database_server_utils as du
import hash_comm as hc
import zmq_comm_node
import rospy
import pdb

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# General configuration variables

# Get the hash length from hash_comm
HASH_LENGTH = hc.Hash.HASH_LENGTH

# When actively polling for an answer or for a changement in variable,
# use this time
CHECK_POLL_TIME=0.1
CHECK_TRIGGER_TIME=0.2
# Timeout value before an answer is considered lost
CHECK_MAX_TIME=1

# Msg codes that are used during the operation of the communication
# channel. Important: all codes should be HEADER_LENGTH characters
HEADER_LENGTH = 5
class Comm_msgs(enum.Enum):
    GHASH = 1
    GDATA = 2
    SHASH = 3
    GERRM = 4
    SERRM = 5

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# SMACH states

class Idle(smach.State):
    def __init__(self, set_client_state, get_sm_shutdown,
                 get_comm_node, sync):
        self.set_client_state = set_client_state
        self.get_sm_shutdown = get_sm_shutdown
        self.sync = sync
        smach.State.__init__(self, outcomes=['to_req_hash',
                                             'to_stopped'])

    def execute(self, userdata):
        self.set_client_state('IDLE')
        while not self.get_sm_shutdown():
            if self.sync.get_state():
                # trigger sync and reset bistable
                return 'to_req_hash'
            rospy.sleep(CHECK_TRIGGER_TIME)
        self.set_client_state('STOPPED')
        return 'to_stopped'

class RequestHash(smach.State):
    def __init__(self, set_client_state, get_comm_node, dbl, get_answer,
            sync):
        self.set_client_state = set_client_state
        self.get_comm_node = get_comm_node
        self.dbl = dbl
        self.get_answer = get_answer
        self.sync = sync
        smach.State.__init__(self,
                             output_keys=['out_answer'],
                             outcomes=['to_idle',
                                       'to_req_hash_reply'])

    def execute(self, userdata):
        self.set_client_state('REQ_HASH')
        # Request current comm node
        comm = self.get_comm_node()
        # Ask server for hash
        msg = Comm_msgs.GHASH.name.encode()
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        while i < int(CHECK_MAX_TIME/CHECK_POLL_TIME):
            answer = self.get_answer()
            if answer is not None:
                userdata.out_answer = answer
                return 'to_req_hash_reply'
            rospy.sleep(CHECK_POLL_TIME)
            i += 1
        self.sync.reset()
        return 'to_idle'

class RequestHashReply(smach.State):
    def __init__(self, set_client_state, dbl, get_answer, sync):
        self.set_client_state = set_client_state
        self.dbl = dbl
        self.get_answer = get_answer
        self.sync = sync
        smach.State.__init__(self, outcomes=['to_idle',
                                             'to_get_data'],
                                   input_keys=['in_answer'],
                                   output_keys=['out_hash_list'])

    def execute(self, userdata):
        self.set_client_state('REQ_HASH_REPLY')
        deserialized = su.deserialize_hashes(userdata.in_answer)
        # print("REQUESTHASH: All ->", deserialized)
        hash_list = su.hashes_not_in_local(self.dbl, deserialized)
        if len(hash_list):
            print("REQUESTHASH: Unique ->", hash_list)
            userdata.out_hash_list = hash_list
            return 'to_get_data'
        self.sync.reset()
        return 'to_idle'

class GetData(smach.State):
    def __init__(self, set_client_state, get_comm_node,
                 dbl, get_answer, sync):
        self.set_client_state = set_client_state
        self.get_comm_node = get_comm_node
        self.dbl = dbl
        self.get_answer = get_answer
        self.sync = sync
        smach.State.__init__(self, outcomes=['to_idle',
                                             'to_get_data_reply'],
                                   input_keys=['in_hash_list'],
                                   output_keys=['out_hash_list',
                                                'out_req_hash',
                                                'out_answer'])

    def execute(self, userdata):
        self.set_client_state('GET_DATA')
        # Request current comm node
        comm = self.get_comm_node()
        hash_list = userdata.in_hash_list.copy()
        req_hash = hash_list.pop()
        print("GETDATA:", req_hash)
        # Ask for hash
        msg = Comm_msgs.GDATA.name.encode() + req_hash.encode()
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        while i < int(CHECK_MAX_TIME/CHECK_POLL_TIME):
            answer = self.get_answer()
            if answer is not None:
                userdata.out_hash_list = userdata.in_hash_list
                userdata.out_answer = answer
                userdata.out_req_hash = req_hash
                return 'to_get_data_reply'
            rospy.sleep(CHECK_POLL_TIME)
            i += 1
        self.sync.reset()
        return 'to_idle'

class GetDataReply(smach.State):
    def __init__(self, set_client_state, dbl, get_answer, sync):
        self.set_client_state = set_client_state
        self.dbl = dbl
        self.get_answer = get_answer
        self.sync = sync
        smach.State.__init__(self, outcomes=['to_idle',
                                             'to_get_more_data'],
                                   input_keys=['in_hash_list',
                                               'in_answer',
                                               'in_req_hash'],
                                   output_keys=['out_hash_list'])
    def execute(self, userdata):
        self.set_client_state('GET_DATA_REPLY')
        # store result in db
        dbm = su.unpack_data(userdata.in_answer)
        hash_list = userdata.in_hash_list.copy()
        print(dbm)
        du.add_modify_data_dbl(self.dbl, dbm)
        hash_list.remove(userdata.in_req_hash)
        print("HASH_LIST", hash_list, "REQ_HASH", userdata.in_req_hash)
        # Transition back
        if hash_list:
            userdata.out_hash_list = hash_list
            return 'to_get_more_data'
        else:
            self.sync.reset()
            return 'to_idle'

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
""" Bistable class. Allows the communication between an external method
in Channel and its state machine. The bistable will be set by a
method in Channel to trigger the synchronization, and it will be reset
inside the state machine """

class Bistable():
    def __init__(self):
        self.state = False
        self.lock = threading.Lock()

    def set(self):
        self.lock.acquire()
        self.state = True
        self.lock.release()

    def reset(self):
        self.lock.acquire()
        self.state = False
        self.lock.release()

    def get_state(self):
        return self.state


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Channel class

class Channel():
    def __init__(self, dbl, this_robot, target_robot, config_file):
        # Basic parameters of the communication channel
        self.robot = this_robot
        self.target_robot = target_robot
        self.dbl = dbl
        # Config file used to fetch configurations
        self.config_file = config_file
        # Variable written by the state machine. Useful to synchronize
        # with specific events
        self.client_state = 'STOPPED'
        # Bistable used to start the synchronization. It will be enabled
        # by self.start_sync(), and it will be disabled inside the state
        # machine once the synchronization starts
        self.sync = Bistable()
        # Change to false before running the state machine. Otherwise,
        # when the SM reaches the idle state it will stop
        self.sm_shutdown = True
        # The answer of the client will be written here
        self.client_answer = None
        # The pointer of the comm node will be stored here
        self.comm_node = None
        # Create state machine and add states
        self.sm = smach.StateMachine(outcomes=['failure', 'stopped'])
        with self.sm:
            smach.StateMachine.add('IDLE',
                                   Idle(self.set_client_state,
                                        self.get_sm_shutdown,
                                        self.get_comm_node,
                                        self.sync),
                    transitions = {'to_req_hash': 'REQ_HASH',
                                   'to_stopped': 'stopped'})
            smach.StateMachine.add('REQ_HASH',
                                    RequestHash(self.set_client_state,
                                                self.get_comm_node,
                                                self.dbl,
                                                self.get_answer,
                                                self.sync),
                transitions = {'to_idle': 'IDLE',
                               'to_req_hash_reply': 'REQ_HASH_REPLY'},
                remapping = {'out_answer': 'sm_answer' })

            smach.StateMachine.add('REQ_HASH_REPLY',
                                    RequestHashReply(self.set_client_state,
                                                     self.dbl,
                                                     self.get_answer,
                                                     self.sync),
                transitions = {'to_idle': 'IDLE',
                               'to_get_data': 'GET_DATA'},
                remapping = {'in_answer': 'sm_answer',
                             'out_hash_list': 'sm_hash_list' })

            smach.StateMachine.add('GET_DATA',
                                   GetData(self.set_client_state,
                                           self.get_comm_node,
                                           self.dbl,
                                           self.get_answer,
                                           self.sync),
                transitions = {'to_idle': 'IDLE',
                               'to_get_data_reply': 'GET_DATA_REPLY'},
                remapping = {'in_hash_list': 'sm_hash_list',
                             'out_hash_list': 'sm_hash_list_2',
                             'out_req_hash': 'sm_req_hash',
                             'out_answer': 'sm_answer_2' })

            smach.StateMachine.add('GET_DATA_REPLY',
                                   GetDataReply(self.set_client_state,
                                                self.dbl,
                                                self.get_answer,
                                                self.sync),
                transitions = {'to_idle': 'IDLE',
                               'to_get_more_data': 'GET_DATA'},
                remapping = {'in_hash_list': 'sm_hash_list_2',
                             'in_req_hash': 'sm_req_hash',
                             'in_answer': 'sm_answer_2',
                             'out_hash_list': 'sm_hash_list' })

    def run(self):
        """ Configures the zmq_comm_node and also starts the state
        machine thread"""
        # The comm node needs to be created first, as it may be required
        # by the SM
        self.comm_node = zmq_comm_node.Comm_node(self.robot,
                                  self.target_robot,
                                  self.config_file,
                                  self.callback_client,
                                  self.callback_server)
        # Unset this flag before starting the SM thread
        self.sm_shutdown = False
        th = threading.Thread(target=self.sm_thread, args=())
        th.setDaemon(True)
        th.start()

    def stop(self):
        # Set the flag and wait until the state machine finishes
        self.sm_shutdown = True
        while self.client_state != "STOPPED":
            rospy.sleep(.05)
        # Close the zmq node
        self.comm_node.terminate()

    def sm_thread(self):
        outcome = self.sm.execute()

    def get_answer(self):
        return self.client_answer

    def get_sm_shutdown(self):
        return self.sm_shutdown

    def get_comm_node(self):
        if not self.comm_node:
            raise Exception("Requesting for an empty comm node")
        return self.comm_node

    def trigger_sync(self):
        if self.sync.get_state():
            raise Exception("Sync has been already requested")
        self.sync.set()

    def set_client_state(self, client_state):
        self.client_state = client_state

    def callback_client(self, msg):
        if not msg is None:
            print("CALLBACK_CLIENT:", self.robot, "- len:", len(msg))
        else:
            print("CALLBACK_CLIENT:", "None")
        self.client_answer = msg

    def callback_server(self, msg):
        print("CALLBACK_SERVER:", self.robot, msg)
        header = msg[:HEADER_LENGTH].decode()
        data = msg[HEADER_LENGTH:]

        if header == Comm_msgs.GHASH.name:
            # Returns all the hashes that this node has
            hashes = su.get_hash_list_from_dbl(self.dbl)
            serialized = su.serialize_hashes(hashes)
            return serialized
        if header == Comm_msgs.GDATA.name:
            r_hash = data.decode()
            # Returns a packed data for the requires hash
            # One hash at a time
            if len(data) != HASH_LENGTH:
                return Comm_msgs.SERRM.name
            try:
                dbm = su.find_hash_dbl(self.dbl, r_hash)
                packed = su.pack_data(dbm)
                return packed
            except Exception:
                return Comm_msgs.SERRM.name
        if header == Comm_msgs.SHASH.name:
            pass
        if header == Comm_msgs.WERRM.name:
            return Comm_msgs.SERRM.name
        return Comm_msgs.SERRM.name
