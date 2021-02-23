#!/usr/bin/env python3

""" test_zmq_comm_node.py
This file tests the basic functionality between two communication nodes.
 - Two nodes are created: robot1 and robot2. The configuration for these are
   defined in CONFIG_FILE.
 - We generate a random message that c1 sends to c2.
 - We check that the message we sent is well received.
"""

import os
import random
import string
import sys
import unittest

import rospkg
import rospy

CONFIG_FILE = "testConfigs/robotConfigs.yml"


class test(unittest.TestCase):
    def test_simple_connection(self):
        self.answer = None

        def cb_c1(value):
            rospy.logdebug(f"CB_C1: {value}")
            self.answer = value

        def cb_s2(value):
            # return the same number we got in the answer
            rospy.logdebug(f"CB_S2: {value}")
            return value

        # Create the two robots
        robot1 = zmq_comm_node.Comm_node(
            "groundstation", "charon", CONFIG_FILE, cb_c1, None
        )
        robot2 = zmq_comm_node.Comm_node(
            "charon", "groundstation", CONFIG_FILE, None, cb_s2
        )

        # Generate random message
        letters = string.ascii_letters
        random_str = "".join(random.choice(letters) for i in range(10))
        random_msg = random_str + str(random.randint(0, 1024))
        random_msg = random_msg.encode()

        # Send message from robot1 to robot 2
        robot1.connect_send_message(random_msg)

        # robot2.connect_send_message(random_msg)

        # Terminate robots and test assertion
        robot1.terminate()
        robot2.terminate()
        self.assertEqual(random_msg, self.answer, "Sent %s" % random_msg)


if __name__ == "__main__":
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("distributed_database")
    scripts_path = os.path.join(pkg_path, "scripts")
    sys.path.append(scripts_path)
    import zmq_comm_node

    # Create a ROS node using during the test
    rospy.init_node("test_zmq_comm_node", log_level=rospy.DEBUG, anonymous=True)

    # Run test cases!
    unittest.main()
