#!/usr/bin/env python3
import enum
import os
import pdb
import signal
import sys
import threading
import uuid

import roslib
import rospkg
import rospy
import yaml
import zmq

import hash_comm

HASH_LENGTH = hash_comm.Hash.HASH_LENGTH


class SyncStatus(enum.Enum):
    IDLE = 0
    SYNCHRONIZING = 1
    FAILURE = 2


class Comm_node:
    def __init__(
        self, this_node, client_node, robot_configs, client_callback, server_callback
    ):
        # Check input arguments
        assert isinstance(this_node, str)
        assert isinstance(client_node, str)
        assert isinstance(robot_configs, dict)

        # Check that this_node and client_node exist in the config file
        if this_node not in robot_configs:
            rospy.logerr(f"{this_node} - Node: this_node not in config file")
            rospy.signal_shutdown("this_node not in config file")
            rospy.spin()
        self.this_node = this_node

        if client_node not in robot_configs[self.this_node]["clients"]:
            rospy.logerr(f"{this_node} - Node: client_node not in config file")
            rospy.signal_shutdown("client_node not in config file")
            rospy.spin()

        self.client_node = client_node

        self.robot_configs = robot_configs

        # The client id is the index of the list of clients
        self.client_id = robot_configs[self.this_node]["clients"].index(client_node)

        # Set network properties
        self.this_port = str(
            int(robot_configs[self.this_node]["base-port"]) + self.client_id)
        self.this_addr = robot_configs[self.this_node]["IP-address"]

        # Configure callbacks
        # We will call:
        # client_callback(data) on successful communication
        # client_callback(None) on erroneous communication (timeout)
        self.client_callback = client_callback
        self.server_callback = server_callback

        # Flag to trigger when an answer is received
        self.answer_received = False

        self.syncStatus = SyncStatus.IDLE
        self.syncStatus_lock = threading.Lock()

        self.context = zmq.Context(1)

        # Start server thread
        server = threading.Thread(target=self.server_thread, args=())
        self.server_running = True
        server.start()

    def connect_send_message(self, msg):
        # TODO keep connection open instead of opening in each call
        SEND_TIMEOUT = 200
        REQUEST_RETRIES = 3

        # Msg check
        if not isinstance(msg, bytes):
            rospy.logdebug(f"{self.this_node} - Node - SENDMSG: msg has to be bytes")
            return

        # Check that we are in the right state
        self.syncStatus_lock.acquire()
        if self.syncStatus != SyncStatus.IDLE:
            rospy.logdebug(f"{self.this_node} - Node - SENDMSG: Sync is running, abort")
            return
        self.client_thread = SyncStatus.SYNCHRONIZING
        self.syncStatus_lock.release()

        # We're all set, send message
        target_robot = self.robot_configs[self.client_node]
        port_offset = target_robot["clients"].index(self.this_node)
        server_endpoint = (
            "tcp://"
            + target_robot["IP-address"]
            + ":"
            + str(int(target_robot["base-port"]) + port_offset)
        )

        rospy.logdebug(
            f"{self.this_node} - Node - SENDMSG: Connecting to server {server_endpoint}"
        )
        client = self.context.socket(zmq.REQ)
        client.connect(server_endpoint)

        poll = zmq.Poller()
        poll.register(client, zmq.POLLIN)

        # Get an uuid for the message to send
        rnd_uuid = str(uuid.uuid4().hex).encode()
        msg_id = hash_comm.Hash(rnd_uuid).digest()
        full_msg = msg_id + msg
        rospy.logdebug(f"{self.this_node} - Node - SENDMSG: Sending ({full_msg})")
        client.send(full_msg)
        start_ts = rospy.Time.now()

        retries_left = REQUEST_RETRIES

        while True:
            socks = dict(poll.poll(SEND_TIMEOUT))
            if socks.get(client) == zmq.POLLIN:
                reply = client.recv()
                if not reply:
                    rospy.logdebug(
                        f"{self.this_node} - Node - SENDMSG: No response from the server"
                    )
                    start_ts = None
                    break
                header = reply[0:HASH_LENGTH]
                data = reply[HASH_LENGTH:]
                if header == msg_id:
                    rospy.logdebug(
                        f"{self.this_node} - Node - SENDMSG: Server replied ({len(reply)} bytes)"
                    )
                    stop_ts = rospy.Time.now()
                    # rospy.loginfo(f"{self.this_node} - Node - SENDMSG: RTT: {(stop_ts - start_ts)/1000000}")
                    start_ts = None
                    self.client_callback(data)
                    break
                else:
                    sys.exit(
                        f"{self.this_node} - Node - SENDMSG: Malformed reply from server: {reply}"
                    )
                    start_ts = None
                    self.client_callback(None)
                    break
            else:
                rospy.logdebug(
                    f"{self.this_node} - Node - SENDMSG: No response from server, retrying..."
                )
                # Socket is confused. Close and remove it.
                client.setsockopt(zmq.LINGER, 0)
                client.close()
                poll.unregister(client)
                retries_left -= 1
                if retries_left == 0:
                    rospy.logdebug(
                        f"{self.this_node} - Node - SENDMSG: Server seems to be offline, abandoning"
                    )
                    self.client_callback(None)
                    break
                rospy.logdebug(
                    f"{self.this_node} - Node - SENDMSG: Reconnecting and resending ({full_msg})"
                )
                # Create new connection
                client = self.context.socket(zmq.REQ)
                client.connect(server_endpoint)
                poll.register(client, zmq.POLLIN)
                client.send(full_msg)
                start_ts = rospy.Time.now()
        client.close()
        self.syncStatus_lock.acquire()
        self.syncStatus = SyncStatus.IDLE
        self.syncStatus_lock.release()

    def server_thread(self):
        RECV_TIMEOUT = 200
        self.server = self.context.socket(zmq.REP)
        self.server.RCVTIMEO = RECV_TIMEOUT

        port = self.this_port

        self.server.bind("tcp://*:" + str(port))

        while self.server_running:
            try:
                request = self.server.recv()
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    rospy.logdebug(
                        f"{self.this_node} - Node - SERVER_RUNNING: {self.server_running}"
                    )
                    continue
                else:
                    sys.exit("SERVER unknown error")
            rospy.logdebug(f"{self.this_node} - Node - SERVER: Got {request}")
            header = request[0:HASH_LENGTH]
            data = request[HASH_LENGTH:]
            reply = self.server_callback(data)
            if reply == None:
                rospy.logerr(f"{self.this_node} - Node - SERVER: reply cannot be none")
                rospy.signal_shutdown("Reply none")
                rospy.spin()
            if not isinstance(reply, bytes):
                rospy.logerr(f"{self.this_node} - Node - SERVER: reply has to be bytes")
                rospy.signal_shutdown("Reply not bytes")
                rospy.spin()
            ans = header + reply
            self.server.send(ans)
            rospy.logdebug(f"{self.this_node} - Node - SERVER: Replied {len(ans)} bytes")
            rospy.logdebug(f"{self.this_node} - SERVER: {ans}")
        self.server.close()
        self.context.term()

    def terminate(self):
        rospy.logdebug(f"{self.this_node} - Node - Terminating server")
        self.server_running = False
