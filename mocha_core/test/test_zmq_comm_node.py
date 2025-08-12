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
import unittest
import yaml
import rospkg
import pdb
import rospy
from colorama import Fore, Style


class Test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        super().setUp()

    def tearDown(self):
        rospy.sleep(1)
        super().tearDown()

    def test_simple_connection(self):
        self.answer = None

        def cb_groundstation(value):
            rospy.logdebug("cb_groundstation")
            self.answer = value

        def cb_charon(value):
            # This function is called upon reception of a message by charon.
            # The return value is transmitted as answer to the original
            # message.
            rospy.logdebug(f"cb_charon: {value}")
            return value

        # Create the two robots
        node_groundstation = zmq_comm_node.Comm_node(
            "basestation", "charon", robot_configs,
            cb_groundstation, cb_groundstation, 2
        )
        node_charon = zmq_comm_node.Comm_node(
            "charon", "basestation", robot_configs,
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


if __name__ == "__main__":
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path("mocha_core")
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import zmq_comm_node

    # Create a ROS node using during the test
    rospy.init_node("test_zmq_comm_node",
                    log_level=rospy.DEBUG, anonymous=False)

    # Get the default path from the ddb_path
    robot_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/robot_configs.yaml")
    # Get the path to the robot config file from the ros parameter robot_configs
    robot_configs = rospy.get_param("robot_configs",
                                    robot_configs_default)

    # Get the yaml dictionary objects
    with open(robot_configs, "r") as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Run test cases!
    unittest.main()
