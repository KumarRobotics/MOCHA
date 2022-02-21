#!/usr/bin/env python3

""" test_zmq_comm_node.py
This file tests the basic functionality between two communication nodes.
 - Two nodes are created: groundstation and charon. The configuration for these are
   defined in CONFIG_FILE. The client of each node is set to the other one.
 - We generate a random message that groundstation sends to charon. Upon reception,
   cb_charon is called. The return value of this function is transmitted to the groundstation.
 - The groundstation receives the messag3e
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

        def cb_groundstation(value):
            rospy.logdebug(f"cb_groundstation: {value}")
            self.answer = value

        def cb_charon(value):
            # This function is called upon reception of a message by charon. The return
            # value is transmitted as answer to the original message.
            rospy.logdebug(f"cb_charon: {value}")
            return value

        # Create the two robots
        node_groundstation = zmq_comm_node.Comm_node(
            "groundstation", "charon", CONFIG_FILE, cb_groundstation, None
        )
        node_charon = zmq_comm_node.Comm_node(
            "charon", "groundstation", CONFIG_FILE, None, cb_charon
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
    pkg_path = rospack.get_path("distributed_database")
    scripts_path = os.path.join(pkg_path, "scripts")
    sys.path.append(scripts_path)
    import zmq_comm_node

    # Create a ROS node using during the test
    rospy.init_node("test_zmq_comm_node", log_level=rospy.DEBUG, anonymous=True)

    # Run test cases!
    unittest.main()
