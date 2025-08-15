#!/usr/bin/env python3
import os
import random
import signal
import sys
import time
import traceback
from functools import partial

import rclpy
import rclpy.logging
import rclpy.time
import yaml
import std_msgs.msg
import subprocess

import mocha_core.database_server as ds
import mocha_core.database_utils as du
import mocha_core.synchronize_channel as sync


def ping(host):
    command = ["ping", "-c", "1", host]
    try:
        result = subprocess.run(command, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception as e:
        print(f"Error pinging {host}: {e}")
        return False


class IntegrateDatabase:
    def __init__(self, node=None):
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Create or use provided node
        if node is None:
            self.node = rclpy.create_node("integrate_database")
        else:
            self.node = node
        
        self.logger = self.node.get_logger()

        # In ROS2, parameters need to be declared before use
        self.node.declare_parameter("robot_name", "")
        self.node.declare_parameter("rssi_threshold", 20)
        self.node.declare_parameter("client_timeout", 6.0)
        self.node.declare_parameter("robot_configs", "")
        self.node.declare_parameter("radio_configs", "")
        self.node.declare_parameter("topic_configs", "")

        self.this_robot = self.node.get_parameter("robot_name").get_parameter_value().string_value
        self.rssi_threshold = self.node.get_parameter("rssi_threshold").get_parameter_value().integer_value
        self.all_channels = []
        self.logger.info(f"{self.this_robot} - Integrate - " +
                        f"RSSI threshold: {self.rssi_threshold}")
        self.client_timeout = self.node.get_parameter("client_timeout").get_parameter_value().double_value
        self.logger.info(f"{self.this_robot} - Integrate - " +
                        f"Client timeout: {self.client_timeout}")

        # Load and check robot configs
        self.robot_configs_file = self.node.get_parameter("robot_configs").get_parameter_value().string_value
        with open(self.robot_configs_file, "r") as f:
            self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        if self.this_robot not in self.robot_configs.keys():
            self.shutdown("Robot not in config file")

        # Load and check radio configs
        self.radio_configs_file = self.node.get_parameter("radio_configs").get_parameter_value().string_value
        with open(self.radio_configs_file, "r") as f:
            self.radio_configs = yaml.load(f, Loader=yaml.FullLoader)
        self.radio = self.robot_configs[self.this_robot]["using-radio"]
        if self.radio not in self.radio_configs.keys():
            self.shutdown("Radio not in config file")

        # Load and check topic configs
        self.topic_configs_file = self.node.get_parameter("topic_configs").get_parameter_value().string_value
        with open(self.topic_configs_file, "r") as f:
            self.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        self.node_type = self.robot_configs[self.this_robot]["node-type"]
        if self.node_type not in self.topic_configs.keys():
            self.shutdown("Node type not in config file")

        # Check that we can ping the radios
        ip = self.robot_configs[self.this_robot]["IP-address"]
        if not ping(ip):
            self.logger.error(f"{self.this_robot} - Integrate - " +
                             f"Cannot ping self {ip}. Is the radio on?")
            self.shutdown("Cannot ping self")
            return

        # Create a database server object with ROS2 node
        self.DBServer = ds.DatabaseServer(self.robot_configs,
                                          self.topic_configs, self.this_robot, node=self.node)

        self.num_robot_in_comm = 0

        # Handle possible interruptions
        self.interrupted = False

        def signal_handler(sig, frame):
            self.logger.warning(f"{self.this_robot} - Integrate - " +
                               f"Got signal. Killing comm nodes.")
            self.interrupted = True
            self.shutdown("Killed by user")
        signal.signal(signal.SIGINT, signal_handler)

        self.logger.info(f"{self.this_robot} - Integrate - " +
                        "Created all communication channels!")

        # Start comm channels with other robots
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
                                   self.client_timeout)
            self.all_channels.append(channel)
            channel.run()

            # Attach a radio trigger to each channel. This will be triggered
            # when the RSSI is high enough. You can use another approach here
            # such as using a timer to periodically trigger the sync
            def make_callback(ch):
                return lambda msg: self.rssi_cb(msg, ch)
            
            self.node.create_subscription(
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

        # Spin!
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass

    def shutdown(self, reason):
        assert isinstance(reason, str)
        self.logger.error(f"{self.this_robot} - Integrate - " + reason)
        for channel in self.all_channels:
            channel.comm_node.terminate()
            self.all_channels.remove(channel)
        self.logger.warning(f"{self.this_robot} - Integrate - " + "Killed Channels")
        if hasattr(self, 'DBServer') and self.DBServer is not None:
            self.DBServer.shutdown()
            self.logger.warning(f"{self.this_robot} - Integrate - " + "Killed DB")
        if rclpy.ok():
            rclpy.shutdown()
        self.logger.warning(f"{self.this_robot} - Integrate - " + "Shutting down")

    def rssi_cb(self, data, comm_node):
        rssi = data.data
        if rssi > self.rssi_threshold:
            self.num_robot_in_comm += 1
            try:
                self.logger.info(f"{self.this_robot} <- {comm_node.target_robot}: Triggering comms")
                comm_node.trigger_sync()
            except:
                traceback.print_exception(*sys.exc_info())


def main():
    # Start the node
    IntegrateDatabase()

if __name__ == "__main__":
    main()
