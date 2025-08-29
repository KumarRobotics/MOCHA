#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading
import pdb
import os
import time
import signal
import yaml
import subprocess
import sys
import queue
import std_msgs.msg
from collections import deque

ON_POSIX = 'posix' in sys.builtin_module_names

def ping(host):
    command = ["ping", "-c", "1", host]
    try:
        result = subprocess.run(command, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception as e:
        print(f"Error pinging {host}: {e}")
        return False

class PeerPublisher():
    # QUEUE_LENGTH is used to filter repeated RSSI
    QUEUE_LENGTH = 3

    def __init__(self, target_name, ros_node):
        assert ros_node is not None
        assert target_name is not None and isinstance(target_name, str)

        self.target_name = target_name
        self.ros_node = ros_node

        self.last_registered_timestamp = None
        self.rssi_ts = []

        self.published_rssi_queue = deque(maxlen=self.QUEUE_LENGTH)
        self.repeated_counter = 0

        # Create a publisher for the node
        topic_name = f"mocha/rajant/rssi/{self.target_name}"
        self.ros_node.get_logger().info(f"{ros_node.this_robot} - Rajant Peer RSSI - Topic /{topic_name} created")
        self.pub = self.ros_node.create_publisher(std_msgs.msg.Int32,
                                                  topic_name, 10)

    def filter_rssi(self, ts, rssi):
        if self.last_registered_timestamp is None:
            self.last_registered_timestamp = ts
        self.rssi_ts.append((ts, rssi))

    def publish_all(self, ts):
        # Skip if we did not collect info for this node in this session
        if self.last_registered_timestamp is None:
           return
        # Check that current end timestamp and msg timestamps agree
        if ts != self.last_registered_timestamp:
            self.ros_node.get_logger().error(f"{self.ros_node.this_robot} - Rajant Peer RSSI - Timestamp of end message different than last registered timestamp in publish_all")
            return
        # Verify that all the timestamps are the same for all the radios
        all_ts = [i[0] for i in self.rssi_ts]
        if not len(all_ts):
            self.ros_node.get_logger().error(f"{self.ros_node.this_robot} - Rajant Peer RSSI - Empty list of timestamps in publish_all.")
            return
        if len(set(all_ts)) != 1:
            self.ros_node.get_logger().error(f"{self.ros_node.this_robot} - Rajant Peer RSSI - Multiple different timestamps for the same group in publish_all.")
            return

        # Find out the largest RSSI and sum of RSSI
        all_rssi = [i[1] for i in self.rssi_ts]
        max_rssi = max(all_rssi)
        sum_rssi = sum(all_rssi)

        # Clear lists
        self.last_registered_timestamp = None
        self.rssi_ts = []

        self.published_rssi_queue.append(sum_rssi)

        # If we published the same sum of RSSI for the last QUEUE_LENGTH times we may drop
        # the subscription. This may happen for two reasons:
        # - It is a dead radio
        # - The RSSI has just not changed
        #
        # As it is difficult to disambiguate one from the other one, a
        # compromise solution is to throttle the topic where we observe this
        # behavior by 1/4.
        #
        # With the current configuration it takes about 30 seconds for a node to
        # disconnect, so this would publish one bad message only
        #
        # print(self.published_rssi_queue, set(self.published_rssi_queue))
        if len(set(self.published_rssi_queue)) == 1 and \
            len(self.published_rssi_queue) == self.QUEUE_LENGTH:
            if self.repeated_counter < 4:
                self.repeated_counter += 1
                self.ros_node.get_logger().debug(f"{self.ros_node.this_robot} - Rajant Peer RSSI - Repeated RSSI for {self.target_name} for the last {self.QUEUE_LENGTH*3} seconds. Throttling counter {self.repeated_counter}")
                return

        self.ros_node.get_logger().debug(f"{self.ros_node.this_robot} - Rajant Peer RSSI - Publishing {self.target_name}")
        msg = std_msgs.msg.Int32()
        msg.data = max_rssi
        self.pub.publish(msg)


class RajantPeerRSSI(Node):
    def __init__(self):
        super().__init__("rajant_peer_rssi")
        self.logger = self.get_logger()

        # Handle shutdown signal
        self.shutdownTriggered = threading.Event()
        self.shutdownTriggered.clear()

        def signal_handler(sig, frame):
            if not self.shutdownTriggered.is_set():
                self.logger.warning(f"{self.this_robot} - Rajant Peer RSSI - Got SIGINT. Triggering shutdown.")
                self.shutdown()
        signal.signal(signal.SIGINT, signal_handler)

        # Declare parameters
        self.declare_parameter("robot_name", "")
        self.declare_parameter("robot_configs", "")
        self.declare_parameter("radio_configs", "")
        self.declare_parameter("bcapi_jar_file", "")

        self.this_robot = self.get_parameter("robot_name").get_parameter_value().string_value

        if len(self.this_robot) == 0:
            self.logger.error(f"{self.this_robot} - Rajant Peer RSSI - Empty robot name")
            raise ValueError("Empty robot name")

        # Load and check robot configs
        self.robot_configs_file = self.get_parameter("robot_configs").get_parameter_value().string_value
        try:
            with open(self.robot_configs_file, "r") as f:
                self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - Rajant Peer RSSI - robot_configs file")
            raise e
        if self.this_robot not in self.robot_configs.keys():
            self.logger.error(f"{self.this_robot} - Rajant Peer RSSI - robot_configs file")
            raise ValueError("Robot not in config file")

        # Load and check radio configs
        self.radio_configs_file = self.get_parameter("radio_configs").get_parameter_value().string_value
        try:
            with open(self.radio_configs_file, "r") as f:
                self.radio_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - Rajant Peer RSSI - radio_configs file")
            raise e
        self.radio = self.robot_configs[self.this_robot]["using-radio"]
        if self.radio not in self.radio_configs.keys():
            self.logger.error(f"{self.this_robot} - Rajant Peer RSSI - radio_configs file")
            raise ValueError("Radio {self.radio} not in config file")

        # Get the location of the jar file for bcapi
        self.bcapi_jar_file = self.get_parameter("bcapi_jar_file").get_parameter_value().string_value
        if not (os.path.isfile(self.bcapi_jar_file) and
                self.bcapi_jar_file.endswith('.jar')):
            self.get_logger().error(f"{self.this_robot} - Rajant Peer RSSI - Erroneous BCAPI jar file")
            raise ValueError(f"Erroneous BCAPI file")

        # Get the target ip for the local rajant
        rajant_name = self.robot_configs[self.this_robot]['using-radio']
        if rajant_name in self.radio_configs.keys():
            self.target_ip = self.radio_configs[rajant_name]['computed-IP-address']
        else:
            self.get_logger().error(f"{self.this_robot} - Rajant Peer RSSI - Radio {rajant_name} for robot {self.this_robot} not found in configs")
            raise ValueError(f"Radio {rajant_name} for robot {self.this_robot} not found in configs")

        # Ping the local rajant
        if not ping(self.target_ip):
            self.get_logger().error(f"{self.this_robot} - Rajant Peer RSSI - Failed to ping {self.target_ip}")
            raise ValueError(f"Failed to ping {self.target_ip}")

        # Create the publishers for the peers
        self.peers = {}
        for peer in self.robot_configs[self.this_robot]["clients"]:
            # Index peer by computed IP
            peer_radio = self.robot_configs[peer]["using-radio"]
            computed_ip = self.radio_configs[peer_radio]["computed-IP-address"]
            self.peers[computed_ip] = PeerPublisher(peer, self)

        # Invert radio lookup
        self.ip_to_radio = {self.radio_configs[radio]["computed-IP-address"]: radio
            for radio in self.radio_configs}

        # Start the java program and thread to read output
        self.start_java_process_and_queue()

        # Thread for processing results
        self.process_thread_shutdown = threading.Event()
        self.process_thread_shutdown.clear()
        self.process_thread = threading.Thread(target=self.process_output, args=())
        self.process_thread.start()

    def shutdown(self):
        if self.shutdownTriggered.is_set():
            return
        self.shutdownTriggered.set()
        # Stop the process_output thread
        self.process_thread_shutdown.set()
        self.process_thread.join()
        # Kill the subprocess
        self.java_process.terminate()
        # This should end the thread for enqueue_output
        self.enqueue_thread.join()

    def enqueue_output(self):
        """ Saves the output of the process in a queue to be parsed
        afterwards """
        for line in self.java_process.stdout:
            self.process_queue.put(line)
        # If the java process dies, we will reach the end of the thread
        self.java_process.stdout.close()

    def process_output(self):
        restart_count = 0
        while not self.process_thread_shutdown.is_set():
            if self.enqueue_thread is not None and not self.enqueue_thread.is_alive():
                time.sleep(1)
                self.get_logger().error(f"{self.this_robot} - Rajant Peer RSSI - Java process died, restarting")
                self.start_java_process_and_queue()
                restart_count += 1
                if restart_count == 5:
                    # shutdown
                    self.get_logger().error(f"{self.this_robot} - Rajant Peer RSSI - Java process died too many times. Killing node.")
                    sys.exit(1)
                continue
            try:
                data = self.process_queue.get(timeout=1)
            except queue.Empty:
                # No data, just continue the loop
                continue
            # Data comes in lines. Decide what to do based on the output
            if data == "\n":
                continue
            elif data == "ERR\n":
                self.java_process.terminate()
                continue
            elif "END," in data:
                # End of transmission, send messages
                data = data.replace("END,", "")
                data = data.replace(";\n", "")
                end_ts = int(data)
                for peer in self.peers:
                    self.peers[peer].publish_all(end_ts)
                continue
            # Process regular messages
            data = data.replace(";\n","") # Remove end line
            ts, iface, peer, rssi = data.split(",")
            # Cleanup things
            ts = int(ts.replace("Ts:", ""))
            iface = iface.replace("Iface:", "")
            peer = peer.replace("Peer:", "")
            rssi = int(rssi.replace("RSSI:", ""))
            # Let the right peer handle it
            if peer in self.peers:
                self.peers[peer].filter_rssi(ts, rssi)
            else:
                # Discover the rogue peer with the radio
                if peer in self.ip_to_radio:
                    rogue_radio = self.ip_to_radio[peer]
                    self.get_logger().debug(f"{self.this_robot} - Rajant Peer RSSI - Peer with radio {rogue_radio} not assigned to any robot")
                else:
                    self.get_logger().debug(f"{self.this_robot} - Rajant Peer RSSI - Peer with IP {peer} not assigned to any robot")



    def start_java_process_and_queue(self):
        # Subprocess to run the java BCAPI interface
        self.get_logger().info(f"{self.this_robot} - Rajant Peer RSSI - Starting Java Process for IP {self.target_ip}")
        # Run process in its own separate process group so it does not get
        # SIGINT. We will handle that ourselves
        popen_kwargs = {}
        popen_kwargs['preexec_fn'] = os.setpgrp
        self.java_process = subprocess.Popen(['java', '-jar', self.bcapi_jar_file,
                   self.target_ip], stdout=subprocess.PIPE, close_fds=ON_POSIX,
                                             text=True, **popen_kwargs)
        self.process_queue = queue.Queue()
        self.enqueue_thread = threading.Thread(target=self.enqueue_output, args=())
        self.enqueue_thread.start()


def main(args=None):
    rclpy.init(args=args)

    try:
        rajant_peer_rssi_node = RajantPeerRSSI()
    except Exception as e:
        print(f"Node initialization failed: {e}")
        rclpy.shutdown()
        return

    # Use mt executor
    mtexecutor = MultiThreadedExecutor(num_threads=4)
    mtexecutor.add_node(rajant_peer_rssi_node)

    # Context manager for clean shutdown
    try:
        while rclpy.ok() and not rajant_peer_rssi_node.shutdownTriggered.is_set():
            mtexecutor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        rajant_peer_rssi_node.shutdown()
        print("Keyboard interrupt")
    except Exception as e:
        print(f"Exception: {e}")
        rajant_peer_rssi_node.shutdown()


if __name__ == "__main__":
    main()
