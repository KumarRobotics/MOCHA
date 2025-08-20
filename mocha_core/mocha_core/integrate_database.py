#!/usr/bin/env python3
import os
import random
import signal
import sys
import time
import pdb
import traceback
from functools import partial

import rclpy
from rclpy.logging import LoggingSeverity
import rclpy.logging
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import yaml
import std_msgs.msg
import subprocess

import mocha_core.database_server as ds
import mocha_core.database_utils as du
import mocha_core.synchronize_channel as sync
from mocha_core.srv import AddUpdateDB, GetDataHeaderDB, SelectDB


def ping(host):
    command = ["ping", "-c", "1", host]
    try:
        result = subprocess.run(command, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception as e:
        print(f"Error pinging {host}: {e}")
        return False


class IntegrateDatabase(Node):
    def __init__(self):
        super().__init__("integrate_database")

        # Handle shutdown signal
        self.interrupted = False
        self.shutdownTriggered = threading.Event()
        self.shutdownTriggered.clear()
        def signal_handler(sig, frame):
            self.logger.warning(f"{self.this_robot} - Integrate - " +
                               f"Got SIGINT. Triggering shutdown.")
            self.interrupted = True
            self.shutdown("Killed by user")
        signal.signal(signal.SIGINT, signal_handler)

        self.logger = self.get_logger()
        # self.logger.set_level(LoggingSeverity.DEBUG)

        # Declare parameters
        self.declare_parameter("robot_name", "")
        self.declare_parameter("rssi_threshold", 20)
        self.declare_parameter("client_timeout", 6.0)
        self.declare_parameter("robot_configs", "")
        self.declare_parameter("radio_configs", "")
        self.declare_parameter("topic_configs", "")

        self.this_robot = self.get_parameter("robot_name").get_parameter_value().string_value
        self.rssi_threshold = self.get_parameter("rssi_threshold").get_parameter_value().integer_value

        if len(self.this_robot) == 0:
            self.shutdown("Empty robot name")

        self.logger.info(f"{self.this_robot} - Integrate - " +
                        f"RSSI threshold: {self.rssi_threshold}")
        self.client_timeout = self.get_parameter("client_timeout").get_parameter_value().double_value
        self.logger.info(f"{self.this_robot} - Integrate - " +
                        f"Client timeout: {self.client_timeout}")

        # Load and check robot configs
        self.robot_configs_file = self.get_parameter("robot_configs").get_parameter_value().string_value
        try:
            with open(self.robot_configs_file, "r") as f:
                self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.shutdown(f"Error opening robot_configs file: {e}")
        if self.this_robot not in self.robot_configs.keys():
            self.shutdown("Robot not in config file")

        # Load and check radio configs
        self.radio_configs_file = self.get_parameter("radio_configs").get_parameter_value().string_value
        try:
            with open(self.radio_configs_file, "r") as f:
                self.radio_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.shutdown(f"Error opening radio_configs file: {e}")
        self.radio = self.robot_configs[self.this_robot]["using-radio"]
        if self.radio not in self.radio_configs.keys():
            self.shutdown("Radio {self.radio} not in config file")

        # Load and check topic configs
        self.topic_configs_file = self.get_parameter("topic_configs").get_parameter_value().string_value
        try:
            with open(self.topic_configs_file, "r") as f:
                self.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.shutdown(f"Error opening topic_configs file: {e}")
        self_type = self.robot_configs[self.this_robot]["node-type"]
        if self_type not in self.topic_configs.keys():
            self.shutdown("Node type not in config file")

        # Check that we can ping the radios
        ip = self.robot_configs[self.this_robot]["IP-address"]
        if not ping(ip):
            self.logger.error(f"{self.this_robot} - Integrate - " +
                             f"Cannot ping self {ip}. Is the radio on?")
            self.shutdown("Cannot ping self")
            return

        # Create database server
        self.DBServer = ds.DatabaseServer(self.robot_configs,
                                          self.topic_configs, self.this_robot, self)


        self.num_robot_in_comm = 0

        self.logger.info(f"{self.this_robot} - Integrate - " +
                        "Created all communication channels!")

        # Start comm channels with other robots
        self.all_channels = []
        self.other_robots = [i for i in list(self.robot_configs.keys()) if i !=
                             self.this_robot]
        for other_robot in self.other_robots:
            if other_robot not in self.robot_configs[self.this_robot]["clients"]:
                self.logger.warning(
                    f"{self.this_robot} - Integrate - "+
                    f"Skipping channel {self.this_robot}->{other_robot} " +
                    "as it is not in the graph of this robot"
                )
                continue
            # Start communication channel
            channel = sync.Channel(self.DBServer.dbl,
                                   self.this_robot,
                                   other_robot, self.robot_configs,
                                   self.client_timeout, self)
            self.all_channels.append(channel)
            channel.run()

            # Attach a radio trigger to each channel. This will be triggered
            # when the RSSI is high enough. You can use another approach here
            # such as using a timer to periodically trigger the sync
            def make_callback(ch):
                return lambda msg: self.rssi_cb(msg, ch)

            self.create_subscription(
                std_msgs.msg.Int32,
                'ddb/rajant/rssi/' + other_robot,
                make_callback(channel),
                10
            )

        # Wait for all the robots to start
        for _ in range(100):
            if self.interrupted or not rclpy.ok():
                self.shutdown("Killed while waiting for other robots")
                return
            time.sleep(0.1)
        # Node is ready
        self.mtexecutor = MultiThreadedExecutor(num_threads=4)
        self.mtexecutor.add_node(self)

    def run(self):
        self.mtexecutor.spin()

    def shutdown(self, reason):
        # Only trigger shutdown once
        if self.shutdownTriggered.is_set():
            sys.exit(1)
            return
        self.shutdownTriggered.set()

        assert isinstance(reason, str)
        self.logger.error(f"{self.this_robot} - Integrate - " + reason)
        # Shutting down communication channels
        if hasattr(self, 'all_channels') and len(self.all_channels) != 0:
            for channel in self.all_channels:
                channel.stop()
                self.all_channels.remove(channel)
            self.logger.warning(f"{self.this_robot} - Integrate - " + "Killed Channels")
            # Wait for all the channels to be gone. This needs to be slightly
            # larger than RECV_TIMEOUT
            time.sleep(3.5)

        pdb.set_trace()

        # Stop the executor to exit the spin loop
        if hasattr(self, 'mtexecutor') and self.mtexecutor is not None:
            self.mtexecutor.shutdown()
            self.logger.warning(f"{self.this_robot} - Integrate - " + "Executor shut down")
        self.destroy_node()
        self.logger.warning(f"{self.this_robot} - Integrate - " + "Node destroyed")
        sys.exit(1)

    def rssi_cb(self, data, comm_node):
        rssi = data.data
        if rssi > self.rssi_threshold:
            self.num_robot_in_comm += 1
            try:
                self.logger.info(f"{self.this_robot} <- {comm_node.target_robot}: Triggering comms")
                comm_node.trigger_sync()
            except:
                traceback.print_exception(*sys.exc_info())


def main(args=None):
    # Initialize ROS2 with command line arguments
    rclpy.init(args=args)

    # Start the node
    try:
        integrate_db = IntegrateDatabase()
    except RuntimeError as e:
        print(f"Node initialization failed: {e}")
        return

    try:
        integrate_db.run()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        pass
    except Exception as e:
        print(e)
    finally:
        integrate_db.shutdown("Shutting down")

if __name__ == "__main__":
    main()
