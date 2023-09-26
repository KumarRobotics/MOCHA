#!/usr/bin/env python3
import os
import random
import signal
import sys
import time
import traceback
from functools import partial

import roslaunch
import rospkg
import rospy
import yaml
import std_msgs.msg
import subprocess

import database_server as ds
import database_utils as du
import synchronize_channel as sync


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
    def __init__(self):
        rospy.init_node("integrate_database", anonymous=False)

        self.this_robot = rospy.get_param("~robot_name")
        self.rssi_threshold = rospy.get_param("~rssi_threshold", 20)
        self.all_channels = []
        rospy.loginfo(f"{self.this_robot} - Integrate - " +
                      f"RSSI threshold: {self.rssi_threshold}")
        self.client_timeout = rospy.get_param("~client_timeout", 6.)
        rospy.loginfo(f"{self.this_robot} - Integrate - " +
                      f"Client timeout: {self.client_timeout}")

        # Load and check robot configs
        self.robot_configs_file = rospy.get_param("~robot_configs")
        with open(self.robot_configs_file, "r") as f:
            self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        if self.this_robot not in self.robot_configs.keys():
            self.shutdown("Robot not in config file")

        # Load and check radio configs
        self.radio_configs_file = rospy.get_param("~radio_configs")
        with open(self.radio_configs_file, "r") as f:
            self.radio_configs = yaml.load(f, Loader=yaml.FullLoader)
        self.radio = self.robot_configs[self.this_robot]["using-radio"]
        if self.radio not in self.radio_configs.keys():
            self.shutdown("Radio not in config file")

        # Load and check topic configs
        self.topic_configs_file = rospy.get_param("~topic_configs")
        with open(self.topic_configs_file, "r") as f:
            self.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        self.node_type = self.robot_configs[self.this_robot]["node-type"]
        if self.node_type not in self.topic_configs.keys():
            self.shutdown("Node type not in config file")

        # Check that we can ping the radios
        ip = self.robot_configs[self.this_robot]["IP-address"]
        if not ping(ip):
            rospy.logerr(f"{self.this_robot} - Integrate - " +
                         f"Cannot ping self {ip}. Is the radio on?")
            rospy.signal_shutdown("Cannot ping self")
            rospy.spin()

        # Create a database server object
        self.DBServer = ds.DatabaseServer(self.robot_configs,
                                          self.topic_configs)

        self.num_robot_in_comm = 0

        # Handle possible interruptions
        self.interrupted = False

        def signal_handler(sig, frame):
            rospy.logwarn(f"{self.this_robot} - Integrate - " +
                          f"Got signal. Killing comm nodes.")
            self.interrupted = True
            rospy.signal_shutdown("Killed by user")
        signal.signal(signal.SIGINT, signal_handler)

        rospy.loginfo(f"{self.this_robot} - Integrate - " +
                      "Created all communication channels!")

        # Start comm channels with other robots
        self.other_robots = [i for i in list(self.robot_configs.keys()) if i !=
                             self.this_robot]
        for other_robot in self.other_robots:
            if other_robot not in self.robot_configs[self.this_robot]["clients"]:
                rospy.logwarn(
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
            rospy.Subscriber('ddb/rajant/rssi/' + other_robot,
                             std_msgs.msg.Int32,
                             self.rssi_cb,
                             channel)

        # Wait for all the robots to start
        for _ in range(100):
            if self.interrupted or rospy.is_shutdown():
                self.shutdown("Killed while waiting for other robots")
                return
            rospy.sleep(.1)

        # Spin!
        rospy.spin()

    def shutdown(self, reason):
        assert isinstance(reason, str)
        rospy.logerr(f"{self.this_robot} - Integrate - " + reason)
        for channel in self.all_channels:
            channel.comm_node.terminate()
            self.all_channels.remove(channel)
        rospy.logwarn(f"{self.this_robot} - Integrate - " + "Killed Channels")
        self.DBServer.shutdown()
        rospy.logwarn(f"{self.this_robot} - Integrate - " + "Killed DB")
        rospy.signal_shutdown(reason)
        rospy.logwarn(f"{self.this_robot} - Integrate - " + "Shutting down")
        rospy.spin()

    def rssi_cb(self, data, comm_node):
        rssi = data.data
        if rssi > self.rssi_threshold:
            self.num_robot_in_comm += 1
            try:
                rospy.loginfo(f"{self.this_robot} <- {comm_node.target_robot}: Triggering comms")
                comm_node.trigger_sync()
            except:
                traceback.print_exception(*sys.exc_info())


if __name__ == "__main__":
    # Start the node
    IntegrateDatabase()
