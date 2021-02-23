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
        self, this_node, client_node, config_file, client_callback, server_callback
    ):

        # Open config file and get list of nodes
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("network_configs")
        yaml_path = os.path.join(package_path, "config", config_file)
        with open(yaml_path, "r") as f:
            cfg = yaml.load(f)

        self.node_list = {}

        # TODO may remove this in the future
        for node in cfg:
            self.node_list[node] = {
                "ip": cfg[node]["IP-address"],
                "base-port": cfg[node]["base-port"],
                "clients": cfg[node]["clients"],
            }

        # Verify is the assigned node is a valid one
        # Get connection port and address
        if this_node not in self.node_list:
            sys.exit("Node %s not found in node list" % this_node)
        self.this_node = this_node

        # Verify that client_node is a valid one, it is not the same as
        # the server, and it is in the client list
        if client_node not in self.node_list:
            sys.exit("Error, robot", client_node, "not in node list")
            return
        if client_node == self.this_node:
            sys.exit("Cannot send to self")
            return
        self.client_node = client_node
        if client_node not in self.node_list[self.this_node]["clients"]:
            sys.exit(
                "Node %s not found in client list of node %s" % (client_node, this_node)
            )

        self.client_node = client_node

        # Set network properties
        port_offset = self.node_list[self.this_node]["clients"][client_node]
        self.this_port = str(
            int(self.node_list[self.this_node]["base-port"]) + port_offset
        )
        self.this_addr = self.node_list[self.this_node]["ip"]

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
        SEND_TIMEOUT = 1500
        REQUEST_RETRIES = 3

        # Msg check
        if not isinstance(msg, bytes):
            rospy.logdebug("SENDMSG: msg has to be bytes")
            return

        # Check that we are in the right state
        self.syncStatus_lock.acquire()
        if self.syncStatus != SyncStatus.IDLE:
            rospy.logdebug("SENDMSG: Sync is running, abort")
            return
        self.client_thread = SyncStatus.SYNCHRONIZING
        self.syncStatus_lock.release()

        # We're all set, send message
        target_robot = self.node_list[self.client_node]
        port_offset = target_robot["clients"][self.this_node]
        server_endpoint = (
            "tcp://"
            + target_robot["ip"]
            + ":"
            + str(int(target_robot["base-port"]) + port_offset)
        )

        rospy.logdebug(f"SENDMSG: Connecting to server {server_endpoint}")
        client = self.context.socket(zmq.REQ)
        client.connect(server_endpoint)

        poll = zmq.Poller()
        poll.register(client, zmq.POLLIN)

        # Get an uuid for the message to send
        rnd_uuid = str(uuid.uuid4().hex).encode()
        msg_id = hash_comm.Hash(rnd_uuid).bindigest()
        full_msg = msg_id + msg
        rospy.logdebug(f"SENDMSG: Sending ({full_msg})")
        client.send(full_msg)

        retries_left = REQUEST_RETRIES

        while True:
            socks = dict(poll.poll(SEND_TIMEOUT))
            if socks.get(client) == zmq.POLLIN:
                reply = client.recv()
                if not reply:
                    rospy.logdebug("SENDMSG: No response from the server")
                    break
                header = reply[0:HASH_LENGTH]
                data = reply[HASH_LENGTH:]
                if header == msg_id:
                    rospy.logdebug(f"SENDMSG: Server replied OK ({len(reply)} bytes)")
                    self.client_callback(data)
                    break
                else:
                    sys.exit(f"SENDMSG: Malformed reply from server: {reply}")
                    self.client_callback(None)
                    break
            else:
                rospy.logdebug("SENDMSG: No response from server, retrying...")
                # Socket is confused. Close and remove it.
                client.setsockopt(zmq.LINGER, 0)
                client.close()
                poll.unregister(client)
                retries_left -= 1
                if retries_left == 0:
                    rospy.logdebug("SENDMSG: Server seems to be offline, abandoning")
                    self.client_callback(None)
                    break
                rospy.logdebug(f"SENDMSG: Reconnecting and resending ({full_msg})")
                # Create new connection
                client = self.context.socket(zmq.REQ)
                client.connect(server_endpoint)
                poll.register(client, zmq.POLLIN)
                client.send(full_msg)
        self.syncStatus_lock.acquire()
        self.syncStatus = SyncStatus.IDLE
        self.syncStatus_lock.release()

    def server_thread(self):
        RECV_TIMEOUT = 1000
        self.server = self.context.socket(zmq.REP)
        self.server.RCVTIMEO = RECV_TIMEOUT

        port = self.this_port

        self.server.bind("tcp://*:" + str(port))

        while self.server_running:
            try:
                request = self.server.recv()
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    rospy.logdebug(f"SERVER_RUNNING: {self.server_running}")
                    continue
                else:
                    sys.exit("SERVER unknown error")
            rospy.logdebug(f"SERVER: Got {request}")
            header = request[0:HASH_LENGTH]
            data = request[HASH_LENGTH:]
            reply = self.server_callback(data)
            if reply == None:
                raise Exception("SERVER: reply cannot be none")
            if not isinstance(reply, bytes):
                raise Exception("SERVER: reply has to be bytes")
            ans = header + reply
            self.server.send(ans)
            rospy.logdebug(f"SERVER: Replied {len(ans)} bytes")
        self.server.close()
        self.context.term()

    def terminate(self):
        rospy.logdebug("Terminating server")
        self.server_running = False


def server_cb(request):
    rospy.logdebug(f"CALLBACK: request {request}")


def signal_handler(sig, frame):
    rospy.logdebug("Handled")
    os._exit(1)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) < 2:
        sys.exit("We need to be somebody. Quitting...")
    this_robot = int(sys.argv[1])
    comm_node = Comm_node(this_robot, server_cb)
    rospy.logdebug(f"This is robot {this_robot}")
    rospy.logdebug(40 * "-")
    while True:
        while True:
            try:
                rts = int(input("KEY_IN: Input robot to sync:\n"))
                if rts < 0:
                    continue
                break
            except:
                pass
        comm_node.set_current_client(rts)
        comm_node.connect_send_message(None)
