#!/usr/bin/env python3

""" test_zmq_comm_node.py
This file tests the basic functionality between two communication nodes.
 - Two nodes are created: basestation and charon. The configuration for these are
   defined in the configuration file. The client of each node is set to the other one.
 - We generate a random message that groundstation sends to charon. Upon reception,
   cb_charon is called. The return value of this function is transmitted to the groundstation.
 - The groundstation receives the message
"""

import os
import random
import string
import sys
import time
import unittest
import yaml
import pdb
import rclpy
import rclpy.logging
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import threading
from colorama import Fore, Style
import mocha_core.zmq_comm_node as zmq_comm_node

class Comm_node_test(Node):
    def __init__(self):
        super().__init__("comm_node_test")


class Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load configurations at the class level
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ddb_path = os.path.join(current_dir, "..")

        # Load robot configs
        robot_configs_path = os.path.join(ddb_path, "config/testConfigs/robot_configs.yaml")
        with open(robot_configs_path, "r") as f:
            cls.robot_configs = yaml.load(f, Loader=yaml.FullLoader)


    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        # Create a mock node to run the tests
        rclpy.init()
        self.comm_node_test = Comm_node_test()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.comm_node_test)
        executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        executor_thread.start()
        super().setUp()

    def tearDown(self):
        time.sleep(1)
        self.executor.shutdown()
        self.comm_node_test.destroy_node()
        rclpy.shutdown()
        super().tearDown()

    def test_simple_connection(self):
        self.answer_cb_gs_client = None
        self.answer_cb_ch_client = None
        logger = self.comm_node_test.get_logger()
        logger.set_level(LoggingSeverity.DEBUG)

        def cb_groundstation_client(value):
            logger.debug(f"cb groundstation client")
            self.answer_cb_gs_client = value

        def cb_groundstation_server(value):
            # This function is called upon reception of a message by the base
            # station
            logger.debug("cb groundstation server")
            to_append = b"GSSERVER"
            return value + to_append

        def cb_charon_client(value):
            logger.debug(f"cb charon client")
            self.answer_cb_ch_client = value

        def cb_charon_server(value):
            # This function is called upon reception of a message by charon.
            # The return value is transmitted as answer to the original
            # message + b"CHARON"
            logger.debug(f"cb charon server")
            to_append = b"CHARONSERVER"
            return value + to_append

        # Create the two robots
        node_groundstation = zmq_comm_node.Comm_node(
            "basestation", "charon", self.robot_configs,
            cb_groundstation_client, cb_groundstation_server,
            2, self.comm_node_test
        )
        node_charon = zmq_comm_node.Comm_node(
            "charon", "basestation", self.robot_configs,
            cb_charon_client, cb_charon_server,
            2, self.comm_node_test
        )

        # Generate random message
        letters = string.ascii_letters
        random_str = "".join(random.choice(letters) for i in range(10))
        random_msg = random_str + str(random.randint(0, 1024))
        random_msg = random_msg.encode()

        # Send message from node_groundstation to robot 2
        node_groundstation.connect_send_message(random_msg)
        node_charon.connect_send_message(random_msg)

        # Terminate robots and test assertion
        node_groundstation.terminate()
        node_charon.terminate()
        self.assertEqual(random_msg + b"CHARONSERVER", self.answer_cb_gs_client,
                         "Sent %s" % random_msg)
        self.assertEqual(random_msg + b"GSSERVER", self.answer_cb_ch_client,
                         "Sent %s" % random_msg)

if __name__ == "__main__":
    # Run test cases!
    unittest.main()
