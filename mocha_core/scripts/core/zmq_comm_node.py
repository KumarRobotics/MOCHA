#!/usr/bin/env python3
import enum
import pdb
import threading
import uuid
import rospy
import zmq
import hash_comm
import mocha_core.msg

HASH_LENGTH = hash_comm.Hash.HASH_LENGTH

SHOW_BANDWIDTH = False


class SyncStatus(enum.Enum):
    IDLE = 0
    SYNCHRONIZING = 1
    FAILURE = 2


class Comm_node:
    def __init__(self, this_node, client_node, robot_configs,
                 client_callback, server_callback, client_timeout):
        # Check input arguments
        assert isinstance(this_node, str)
        assert isinstance(client_node, str)
        assert isinstance(robot_configs, dict)
        assert callable(client_callback)
        assert callable(server_callback)
        assert isinstance(client_timeout, (int, float))

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
        self.client_timeout = client_timeout

        # Create a publisher for the client bandwidth
        self.pub_client_stats = rospy.Publisher(f"ddb/client_stats/{self.client_node}",
                                                mocha_core.msg.Client_stats,
                                                queue_size=10)
        self.pub_client_count = 0

        self.syncStatus = SyncStatus.IDLE
        self.syncStatus_lock = threading.Lock()

        self.context = zmq.Context(1)

        # Start server thread
        server = threading.Thread(target=self.server_thread, args=())
        self.server_running = True
        server.start()

    def connect_send_message(self, msg):
        # TODO keep connection open instead of opening in each call
        # Msg check
        if not isinstance(msg, bytes):
            rospy.logdebug(f"{self.this_node} - Node - SENDMSG: " +
                           "msg has to be bytes")
            return

        # Check that we are in the right state
        self.syncStatus_lock.acquire()
        if self.syncStatus != SyncStatus.IDLE:
            rospy.logdebug(f"{self.this_node} - Node - SENDMSG: " +
                           "Sync is running, abort")
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

        rospy.logdebug(f"{self.this_node} - Node - SENDMSG: " +
                       f"Connecting to server {server_endpoint}")
        client = self.context.socket(zmq.REQ)
        client.connect(server_endpoint)

        poll = zmq.Poller()
        poll.register(client, zmq.POLLIN)

        # Get an uuid for the message to send
        rnd_uuid = str(uuid.uuid4().hex).encode()
        msg_id = hash_comm.Hash(rnd_uuid).digest()
        full_msg = msg_id + msg
        rospy.logdebug(f"{self.this_node} - Node - SENDMSG: " +
                       f"Sending ({full_msg})")
        client.send(full_msg)
        start_ts = rospy.Time.now()
        # Wait at most self.client_timeout * 1000 ms
        socks = dict(poll.poll(self.client_timeout*1000))
        if socks.get(client) == zmq.POLLIN:
            reply = client.recv()
            if not reply:
                rospy.logdebug(
                    f"{self.this_node} - Node - SENDMSG: " +
                    "No response from the server"
                )
                self.client_callback(None)
            else:
                header = reply[0:HASH_LENGTH]
                data = reply[HASH_LENGTH:]
                if header == msg_id:
                    rospy.logdebug(
                        f"{self.this_node} - Node - SENDMSG: Server replied " +
                        f"({len(reply)} bytes)"
                    )
                    stop_ts = rospy.Time.now()
                    time_d = stop_ts - start_ts
                    time_s = float(time_d.to_sec())
                    bw = len(reply)/time_s/1024/1024
                    stats = mocha_core.msg.Client_stats()
                    stats.header.stamp = rospy.Time.now()
                    stats.header.frame_id = self.this_node
                    stats.header.seq = self.pub_client_count
                    self.pub_client_count += 1
                    stats.msg = msg[:5].decode("utf-8")
                    stats.rtt = time_s
                    stats.bw = bw
                    stats.answ_len = len(reply)
                    self.pub_client_stats.publish(stats)
                    if len(reply) > 10*1024 and SHOW_BANDWIDTH:
                        rospy.loginfo(f"{self.this_node} - Node - " +
                                      f"SENDMSG: Data RTT: {time_s}")
                        rospy.loginfo(f"{self.this_node} - Node - SENDMSG: " +
                                      f"BW: {bw}" +
                                      "MBytes/s")
                    self.client_callback(data)
                else:
                    rospy.logerr(f"{self.this_node} - Node -  SENDMSG: " +
                                 f"Malformed reply from server: {reply}")
                    self.client_callback(None)
        else:
            rospy.logdebug(f"{self.this_node} - Node - SENDMSG: " +
                           "No response from server")
            self.client_callback(None)
        client.setsockopt(zmq.LINGER, 0)
        client.close()
        poll.unregister(client)
        self.syncStatus_lock.acquire()
        self.syncStatus = SyncStatus.IDLE
        self.syncStatus_lock.release()

    def server_thread(self):
        # This timer does not have a big impact as it is only the timer until
        # the recv times out Most calls from the client are very lightweight
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
                    # rospy.logdebug(
                    #     f"{self.this_node} - Node - SERVER_RUNNING: " +
                    #     f"{self.server_running}"
                    # )
                    continue
                else:
                    rospy.logerr(f"{self.this_node} - Node - SERVER: " +
                                 f"ZMQ error: {e}")
                    rospy.signal_shutdown("ZMQ error")
                    rospy.spin()
            rospy.logdebug(f"{self.this_node} - Node - SERVER: Got {request}")
            header = request[0:HASH_LENGTH]
            data = request[HASH_LENGTH:]
            reply = self.server_callback(data)
            if reply is None:
                rospy.logerr(f"{self.this_node} - Node - SERVER: " +
                             f"reply cannot be none")
                rospy.signal_shutdown("Reply none")
                rospy.spin()
            if not isinstance(reply, bytes):
                rospy.logerr(f"{self.this_node} - Node - SERVER: " +
                             "reply has to be bytes")
                rospy.signal_shutdown("Reply not bytes")
                rospy.spin()
            ans = header + reply
            self.server.send(ans)
            rospy.logdebug(f"{self.this_node} - Node - SERVER: " +
                           "Replied {len(ans)} bytes")
            rospy.logdebug(f"{self.this_node} - SERVER: {ans}")
        self.server.close()
        self.context.term()

    def terminate(self):
        rospy.logdebug(f"{self.this_node} - Node - Terminating server")
        self.server_running = False
