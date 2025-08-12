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
from colorama import Fore, Style


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
        super().setUp()

    def tearDown(self):
        time.sleep(1)
        super().tearDown()

    def test_simple_connection(self):
        self.answer = None
        logger = rclpy.logging.get_logger('test_zmq_comm_node')

        def cb_groundstation(value):
            logger.debug("cb_groundstation")
            self.answer = value

        def cb_charon(value):
            # This function is called upon reception of a message by charon.
            # The return value is transmitted as answer to the original
            # message.
            logger.debug(f"cb_charon: {value}")
            return value

        # Create the two robots
        node_groundstation = zmq_comm_node.Comm_node(
            "basestation", "charon", self.robot_configs,
            cb_groundstation, cb_groundstation, 2
        )
        node_charon = zmq_comm_node.Comm_node(
            "charon", "basestation", self.robot_configs,
            cb_charon, cb_charon, 2
        )

        # Generate random message
        letters = string.ascii_letters
        random_str = "".join(random.choice(letters) for i in range(10))
        random_msg = random_str + str(random.randint(0, 1024))
        random_msg = random_msg.encode()

        # Send message from node_groundstation to robot 2
        node_groundstation.connect_send_message(random_msg)

        # node_charon.connect_send_message(random_msg)

        # Terminate robots and test assertion
        node_groundstation.terminate()
        node_charon.terminate()
        self.assertEqual(random_msg, self.answer, "Sent %s" % random_msg)


# Add the mocha_core module path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
mocha_core_path = os.path.join(current_dir, "..", "mocha_core")
sys.path.append(mocha_core_path)

import zmq_comm_node

if __name__ == "__main__":
    # Run test cases!
    unittest.main()
