#!/usr/bin/env python3

import enum
import threading
import time
import smach
import mocha_core.database as db
import mocha_core.database_utils as du
import mocha_core.hash_comm as hc
import mocha_core.zmq_comm_node as zmq_comm_node
import rclpy
import rclpy.logging
import rclpy.time
import pdb
from builtin_interfaces.msg import Time
from mocha_core.msg import SMState

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# General configuration variables

# Get the header length from hash_comm
HEADER_LENGTH = hc.TsHeader.HEADER_LENGTH

# When actively polling for an answer or for a changement in variable,
# use this time
CHECK_POLL_TIME = 0.2
CHECK_TRIGGER_TIME = 0.05

# Msg codes that are used during the operation of the communication
# channel. Important: all codes should be CODE_LENGTH characters
CODE_LENGTH = 5


class Comm_msgs(enum.Enum):
    GHEAD = 1
    GDATA = 2
    DENDT = 3
    SERRM = 4

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
""" SMACH states for the synchronization state machine"""


class Idle(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_req_hash',
                                             'to_stopped'])

    def execute(self, userdata):
        self.outer.publishState("Idle Start")
        while (not self.outer.sm_shutdown.is_set()):
            if self.outer.sync.get_state():
                # trigger sync and reset bistable
                self.outer.publishState("Idle to RequestHash")
                return 'to_req_hash'
            time.sleep(CHECK_TRIGGER_TIME)
        self.outer.publishState("Idle to Stopped")
        return 'to_stopped'


class RequestHash(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self,
                             output_keys=['out_answer'],
                             outcomes=['to_idle',
                                       'to_req_hash_reply',
                                       'to_stopped'])

    def execute(self, userdata):
        self.outer.publishState("RequestHash Start")
        # Request current comm node
        comm = self.outer.get_comm_node()
        # Ask server for hash
        msg = Comm_msgs.GHEAD.name.encode()
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        # Important: the <= is not a typo. We want one iteration more of the
        # loop to wait for the timeout
        while (i <= int(self.outer.client_timeout/CHECK_POLL_TIME)
               and not self.outer.sm_shutdown.is_set()):
            answer = self.outer.client_answer
            if answer is not None:
                self.outer.logger.debug(f"{comm.this_node} - Channel" +
                               f"- REQUESTHASH: {answer}")
                userdata.out_answer = answer
                self.outer.publishState("RequestHash to Reply")
                return 'to_req_hash_reply'
            time.sleep(CHECK_POLL_TIME)
            i += 1
        if self.outer.sm_shutdown.is_set():
            self.outer.publishState("RequestHash to Stopped")
            return 'to_stopped'
        self.outer.sync.reset()
        self.outer.publishState("RequestHash to Idle")
        return 'to_idle'


class RequestHashReply(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_transmission_end',
                                             'to_get_data'],
                             input_keys=['in_answer'],
                             output_keys=['out_hash_list'])

    def execute(self, userdata):
        self.outer.publishState("GetHashReply Start")
        deserialized = du.deserialize_headers(userdata.in_answer)
        # print("REQUESTHASH: All ->", deserialized)
        # FIXME(fernando): Configure this depending on the message type
        # depending on the message type
        hash_list = self.outer.dbl.headers_not_in_local(deserialized,
                                                        newer=True)
        self.outer.logger.debug(f"======== - REQUESTHASH: {hash_list}")
        if len(hash_list):
            # We have hashes. Go get them
            # rospy.logdebug(f"{self.this_robot} - REQUESTHASH: Unique -> {hash_list}")
            userdata.out_hash_list = hash_list
            self.outer.publishState("GetHashReply to GetData")
            return 'to_get_data'
        # We have no hashes. Sync is complete
        self.outer.publishState("GetHashReply to TransmissionEnd")
        return 'to_transmission_end'


class GetData(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_idle',
                                             'to_get_data_reply',
                                             'to_stopped'],
                             input_keys=['in_hash_list'],
                             output_keys=['out_hash_list',
                                          'out_req_hash',
                                          'out_answer'])

    def execute(self, userdata):
        self.outer.publishState("GetData Start")
        # Request current comm node
        comm = self.outer.get_comm_node()
        hash_list = userdata.in_hash_list.copy()
        # Get the first hash of the list, the one with the higher priority
        req_hash = hash_list.pop(0)
        self.outer.logger.debug(f"{comm.this_node} - Channel - GETDATA: {req_hash}")
        # Ask for hash
        msg = Comm_msgs.GDATA.name.encode() + req_hash
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        # Important: the <= is not a typo. We want one iteration more of the
        # loop to wait for the timeout
        while (i <= int(self.outer.client_timeout/CHECK_POLL_TIME)
               and not self.outer.sm_shutdown.is_set()):
            answer = self.outer.client_answer
            if answer is not None:
                userdata.out_hash_list = userdata.in_hash_list
                userdata.out_answer = answer
                userdata.out_req_hash = req_hash
                self.outer.publishState("GetData to GetDataReply")
                return 'to_get_data_reply'
            time.sleep(CHECK_POLL_TIME)
            i += 1
        if self.outer.sm_shutdown.is_set():
            self.outer.publishState("GetData to Stopped")
            return 'to_stopped'
        self.outer.sync.reset()
        self.outer.publishState("GetData to Idle")
        return 'to_idle'


class GetDataReply(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_transmission_end',
                                             'to_get_more_data'],
                             input_keys=['in_hash_list',
                                         'in_answer',
                                         'in_req_hash'],
                             output_keys=['out_hash_list'])

    def execute(self, userdata):
        self.outer.publishState("GetDataReply Start")
        # store result in db
        dbm = du.unpack_data(userdata.in_req_hash, userdata.in_answer)
        hash_list = userdata.in_hash_list.copy()
        self.outer.dbl.add_modify_data(dbm)
        hash_list.remove(userdata.in_req_hash)
        # rospy.logdebug(f"HASH_LIST {hash_list} REQ_HASH {userdata.in_req_hash}")
        # Transition back
        if hash_list:
            userdata.out_hash_list = hash_list
            self.outer.publishState("GetDataReply to GetMoreData")
            return 'to_get_more_data'
        else:
            self.outer.publishState("GetDataReply to TransmissionEnd")
            return 'to_transmission_end'


class TransmissionEnd(smach.State):
    def __init__(self, outer):
        self.outer = outer
        smach.State.__init__(self, outcomes=['to_idle',
                                             'to_stopped'])

    def execute(self, userdata):
        self.outer.publishState("TransmissionEnd Start")
        # Request current comm node
        comm = self.outer.get_comm_node()
        self.outer.logger.debug(f"{comm.this_node} - Channel - DENDT")
        # Ask for hash
        msg = Comm_msgs.DENDT.name.encode() + self.outer.this_robot.encode()
        comm.connect_send_message(msg)
        # Wait for an answer in a polling fashion
        i = 0
        # Important: the <= is not a typo. We want one iteration more of the
        # loop to wait for the timeout
        while (i <= int(self.outer.client_timeout/CHECK_POLL_TIME)
               and not self.outer.sm_shutdown.is_set()):
            answer = self.outer.client_answer
            if answer is not None:
                # We received an ACK
                if self.outer.client_sync_complete_pub is not None:
                    current_time = self.outer.ros_node.get_clock().now()
                    self.outer.client_sync_complete_pub.publish(current_time.to_msg())
                break
            time.sleep(CHECK_POLL_TIME)
            i += 1
        if self.outer.sm_shutdown.is_set():
            # self.outer.publishState("TransmissionEnd to Stopped")
            return 'to_stopped'
        self.outer.sync.reset()
        self.outer.publishState("TransmissionEnd to Idle")
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
    def __init__(self, dbl, this_robot,
                 target_robot, robot_configs,
                 client_timeout, ros_node):
        # Check input arguments
        assert type(dbl) is db.DBwLock
        assert type(this_robot) is str
        assert type(target_robot) is str
        assert type(robot_configs) is dict
        assert type(client_timeout) is float or type(client_timeout) is int

        # Confirm that we are providing ad ROS node
        assert ros_node is not None
        self.ros_node = ros_node
        self.logger = self.ros_node.get_logger()
        self.ros_node_name = self.ros_node.get_fully_qualified_name()

        # Override smach logger to use ROS2 loggers
        smach.set_loggers(self.logger.debug, self.logger.warn,
                          self.logger.debug, self.logger.error)

        # Basic parameters of the communication channel
        self.this_robot = this_robot
        self.target_robot = target_robot
        self.dbl = dbl
        # Config file used to fetch configurations
        self.robot_configs = robot_configs

        # Client timeout defines the time before an answer is considered lost
        self.client_timeout = client_timeout

        # Bistable used to start the synchronization. It will be enabled
        # by self.start_sync(), and it will be disabled inside the state
        # machine once the synchronization starts
        self.sync = Bistable()

        # Change to false before running the state machine. Otherwise,
        # when the SM reaches the idle state it will stop
        self.sm_shutdown = threading.Event()
        self.sm_shutdown.set()
        # The answer of the client will be written here
        self.client_answer = None
        # The pointer of the comm node will be stored here
        self.comm_node = None
        # Create state machine and add states
        self.sm = smach.StateMachine(outcomes=['failure', 'stopped'])

        # Create topic to notify that the transmission ended
        self.client_sync_complete_pub = self.ros_node.create_publisher(
            Time,
            f"{self.ros_node_name}/client_sync_complete/{self.target_robot}",
            20
        )
        self.server_sync_complete_pub = self.ros_node.create_publisher(
            Time,
            f"{self.ros_node_name}/server_sync_complete/{self.target_robot}",
            20
        )

        # Create a topic that prints the current state of the state machine
        self.sm_state_pub = self.ros_node.create_publisher(
            SMState,
            f"{self.ros_node_name}/client_sm_state/{self.target_robot}",
            20
        )

        with self.sm:
            smach.StateMachine.add('IDLE',
                                   Idle(self),
                                   transitions={'to_req_hash': 'REQ_HASH',
                                                'to_stopped': 'stopped'})
            smach.StateMachine.add('REQ_HASH',
                                   RequestHash(self),
                                   transitions={'to_idle': 'IDLE',
                                                'to_req_hash_reply': 'REQ_HASH_REPLY',
                                                'to_stopped': 'stopped'},
                                   remapping={'out_answer': 'sm_answer'})

            smach.StateMachine.add('REQ_HASH_REPLY',
                                   RequestHashReply(self),
                                   transitions={'to_transmission_end': 'TRANSMISSION_END',
                                                'to_get_data': 'GET_DATA'},
                                   remapping={'in_answer': 'sm_answer',
                                              'out_hash_list': 'sm_hash_list'})

            smach.StateMachine.add('GET_DATA',
                                   GetData(self),
                                   transitions={'to_idle': 'IDLE',
                                                'to_get_data_reply': 'GET_DATA_REPLY',
                                                'to_stopped': 'stopped'},
                                   remapping={'in_hash_list': 'sm_hash_list',
                                              'out_hash_list': 'sm_hash_list_2',
                                              'out_req_hash': 'sm_req_hash',
                                              'out_answer': 'sm_answer_2' })

            smach.StateMachine.add('GET_DATA_REPLY',
                                   GetDataReply(self),
                                   transitions={'to_transmission_end': 'TRANSMISSION_END',
                                                'to_get_more_data': 'GET_DATA'},
                                   remapping={'in_hash_list': 'sm_hash_list_2',
                                              'in_req_hash': 'sm_req_hash',
                                              'in_answer': 'sm_answer_2',
                                              'out_hash_list': 'sm_hash_list'})

            smach.StateMachine.add('TRANSMISSION_END',
                                   TransmissionEnd(self),
                                   transitions={'to_idle': 'IDLE',
                                                'to_stopped': 'stopped'})

    def publishState(self, msg):
        """ Publish the string msg (where the state message will be stored)
        with a timestamp"""
        assert type(msg) is str
        if self.sm_state_pub is not None:
            state_msg = SMState()
            state_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            state_msg.header.frame_id = self.this_robot
            state_msg.state = msg
            self.sm_state_pub.publish(state_msg)

    def run(self):
        """ Configures the zmq_comm_node and also starts the state
        machine thread"""
        # The comm node needs to be created first, as it may be required
        # by the SM
        self.comm_node = zmq_comm_node.Comm_node(self.this_robot,
                                                 self.target_robot,
                                                 self.robot_configs,
                                                 self.callback_client,
                                                 self.callback_server,
                                                 self.client_timeout,
                                                 self.ros_node)
        # Unset this flag before starting the SM thread
        self.sm_shutdown.clear()
        self.th = threading.Thread(target=self.sm_thread, args=())
        self.th.start()

    def stop(self):
        # Set the flag and wait until the state machine finishes
        self.sm_shutdown.set()
        # This terminates the comm node, so we can wait for the thread to finish
        self.th.join()
        self.logger.info(f"Channel {self.this_robot} -> {self.target_robot} destroyed")


    def sm_thread(self):
        # Start the state machine and wait until it ends
        self.logger.info(f"Channel {self.this_robot} -> {self.target_robot} started")
        outcome = self.sm.execute()
        # Terminate the comm node once the state machine ends
        self.comm_node.terminate()
        exit_msg = f"Channel {self.this_robot} -> {self.target_robot}" + \
            f" finished with outcome: {outcome}"
        if outcome == 'failure':
            self.logger.error(exit_msg)
        elif outcome == 'stopped':
            self.logger.info(exit_msg)

    def get_comm_node(self):
        if not self.comm_node:
            self.logger.error("Requesting for an empty comm node")
            raise RuntimeError("Requesting for an empty comm node")
        return self.comm_node

    def trigger_sync(self):
        if self.sync.get_state():
            self.logger.warn(f"{self.this_robot} <- {self.target_robot}: Channel busy")
        else:
            self.sync.set()

    def callback_client(self, msg):
        if msg is not None:
            self.logger.debug(f"{self.this_robot} - Channel - CALLBACK_CLIENT: len: {len(msg)}")
        else:
            self.logger.debug(f"{self.this_robot} - Channel - CALLBACK_CLIENT - None")
        self.client_answer = msg

    def callback_server(self, msg):
        self.logger.debug(f"{self.this_robot} - Channel - CALLBACK_SERVER: {msg}")
        header = msg[:CODE_LENGTH].decode()
        data = msg[CODE_LENGTH:]

        if header == Comm_msgs.GHEAD.name:
            # Returns all the headers that this node has
            # FIXME(Fernando): Configure this depending on the message type
            headers = self.dbl.get_header_list(filter_latest=True)
            self.logger.debug(f"{self.this_robot} - Channel - Sending {len(headers)} headers")
            self.logger.debug(f"{self.this_robot} - Channel - {headers}")
            serialized = du.serialize_headers(headers)
            return serialized
        if header == Comm_msgs.GDATA.name:
            r_header = data
            # Returns a packed data for the requires header
            # One header at a time
            if len(data) != HEADER_LENGTH:
                self.logger.error(f"{self.this_robot} - Wrong header length: {len(data)}")
                return Comm_msgs.SERRM.name.encode()
            try:
                dbm = self.dbl.find_header(r_header)
                packed = du.pack_data(dbm)
                return packed
            except Exception:
                self.logger.error(f"{self.this_robot} - Header not found: {r_header}")
                return Comm_msgs.SERRM.name.encode()
        if header == Comm_msgs.DENDT.name:
            target = data
            if target.decode() != self.target_robot:
                print(f"{self.this_robot} - Channel - Wrong DENDT -" +
                             f" Target: {target.decode()} - " +
                             f"My target: {self.target_robot}")
            if self.server_sync_complete_pub is not None:
                current_time = self.ros_node.get_clock().now()
                self.server_sync_complete_pub.publish(current_time.to_msg())
            return "Ack".encode()
        return Comm_msgs.SERRM.name.encode()
