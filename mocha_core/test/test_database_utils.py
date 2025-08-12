#!/usr/bin/env python3
import unittest
import sys
import os
from pprint import pprint
import uuid
import geometry_msgs.msg
import rospkg
import pdb
import rospy
from colorama import Fore, Back, Style
import yaml
import random

ROBOT0_TOPIC2_PRIO0 = b'\x00\x00\x00u\x00\xde'
ROBOT1_TOPIC1_PRIO4 = b'\x01\x00\x01O\x00\xdc'
ROBOT1_TOPIC2_PRIO2 = b'\x01\x00\x01O\x00\xc7'


class test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        super().setUp()

    def tearDown(self):
        rospy.sleep(1)
        super().tearDown()

    def test_serialize_deserialize(self):
        for _ in range(10):
            header_list = []
            for _ in range(20):
                header_list.append(du.generate_random_header())
            serialized = du.serialize_headers(header_list)
            deserialized = du.deserialize_headers(serialized)
            self.assertEqual(len(header_list), len(deserialized))
            self.assertEqual(deserialized, header_list)

    def test_deserialize_wrong(self):
        header_list = []
        for _ in range(5):
            header_list.append(du.generate_random_header())
        serialized = du.serialize_headers(header_list)
        with self.assertRaises(Exception):
            du.deserialize_headers(serialized[1:])

    def test_pack_unpack_data_topic(self):
        dbl = sample_db.get_sample_dbl()
        dbm = dbl.find_header(ROBOT1_TOPIC1_PRIO4)
        packed = du.pack_data(dbm)
        u_dbm = du.unpack_data(ROBOT1_TOPIC1_PRIO4, packed)
        self.assertEqual(dbm, u_dbm)
        dbm = dbl.find_header(ROBOT1_TOPIC2_PRIO2)
        packed = du.pack_data(dbm)
        u_dbm = du.unpack_data(ROBOT1_TOPIC2_PRIO2, packed)
        self.assertEqual(dbm, u_dbm)

    def test_topic_id(self):
        for i in range(50):
            # Pick a random robot
            robot = random.choice(list(robot_configs.keys()))
            # Pick a random topic
            topic_list = topic_configs[robot_configs[robot]["node-type"]]
            topic = random.choice(topic_list)
            id = du.get_topic_id_from_name(robot_configs, topic_configs,
                                           robot, topic["msg_topic"])
            topic_find = du.get_topic_name_from_id(robot_configs,
                                                   topic_configs, robot, id)
            self.assertEqual(topic["msg_topic"], topic_find)

    def test_robot_id(self):
        for robot in robot_configs:
            number = du.get_robot_id_from_name(robot_configs, robot)
            robot_name = du.get_robot_name_from_id(robot_configs, number)
            self.assertEqual(robot, robot_name)

if __name__ == '__main__':
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('mocha_core')
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import database_utils as du
    import sample_db
    import hash_comm as hc

    # Set the node name
    rospy.init_node('test_synchronize_utils', anonymous=False)

    # Get the default path from the ddb_path
    robot_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/robot_configs.yaml")
    # Get the path to the robot config file from the ros parameter robot_configs
    robot_configs = rospy.get_param("robot_configs",
                                    robot_configs_default)

    # Get the yaml dictionary objects
    with open(robot_configs, "r") as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Get the default path from the ddb_path
    topic_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/topic_configs.yaml")
    # Get the path to the robot config file from the ros
    # parameter topic_configs
    topic_configs = rospy.get_param("topic_configs",
                                    topic_configs_default)

    # Get the yaml dictionary objects
    with open(topic_configs, "r") as f:
        topic_configs = yaml.load(f, Loader=yaml.FullLoader)

    msg_types = du.msg_types(robot_configs, topic_configs)

    # Run test cases!
    unittest.main()
