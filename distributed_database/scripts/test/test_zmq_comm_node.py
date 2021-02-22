#!/usr/bin/env python3

import os
import unittest
import sys
import random
import yaml
import rospkg

CONFIG_FILE = "testConfigs/robotConfigs.yml"

class test(unittest.TestCase):
    def test_simple_connection(self):
        self.answer = None
        def cb_c1(value):
            print('CB_C1: %s' % value)
            self.answer = value

        def cb_s2(value):
            # return the same number we got in the answer
            print('CB_S2: %s' % value)
            return value
        robot1 = zmq_comm_node.Comm_node('groundstation', 
                                         'charon',
                                         CONFIG_FILE,
                                         cb_c1, None)
        robot2 = zmq_comm_node.Comm_node('charon',
                                         'groundstation',
                                         CONFIG_FILE,
                                         None, cb_s2)
        random_msg = "hoho" + str(random.randint(0, 1024))
        random_msg = random_msg.encode()
        robot1.connect_send_message(random_msg)

        #robot2.connect_send_message(random_msg)

        robot1.terminate()
        robot2.terminate()
        self.assertEqual(random_msg, self.answer, 'Sent %s' % random_msg)

if __name__ == '__main__':
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(pkg_path, "scripts")
    sys.path.append(scripts_path)
    import zmq_comm_node

    # Run test cases!
    unittest.main()
