#!/usr/bin/env python3
import unittest
import sys
import os
from pprint import pprint
import time
import rclpy
from colorama import Fore, Back, Style
import threading
import yaml
import random
from rclpy.logging import LoggingSeverity
import mocha_core.database_utils as du
import sample_db
import mocha_core.hash_comm as hc
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

ROBOT0_TOPIC2_PRIO0 = b'\x00\x00\x00u\x00\xde'
ROBOT1_TOPIC1_PRIO4 = b'\x01\x00\x01O\x00\xdc'
ROBOT1_TOPIC2_PRIO2 = b'\x01\x00\x01O\x00\xc7'

class Database_utils_test(Node):
    def __init__(self):
        super().__init__("test_database_utils")


class test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load configurations at the class level
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ddb_path = os.path.join(current_dir, "..")

        # Load robot configs
        robot_configs_path = os.path.join(ddb_path, "config/testConfigs/robot_configs.yaml")
        with open(robot_configs_path, "r") as f:
            cls.robot_configs = yaml.load(f, Loader=yaml.FullLoader)

        # Load topic configs
        topic_configs_path = os.path.join(ddb_path, "config/testConfigs/topic_configs.yaml")
        with open(topic_configs_path, "r") as f:
            cls.topic_configs = yaml.load(f, Loader=yaml.FullLoader)


    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        rclpy.init()
        self.test_ros_node = Database_utils_test()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.test_ros_node)
        self.logger = self.test_ros_node.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)
        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs,
                                     self.test_ros_node)
        executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        executor_thread.start()
        super().setUp()

    def tearDown(self):
        time.sleep(1)
        self.executor.shutdown()
        self.test_ros_node.destroy_node()
        rclpy.shutdown()
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
        self.assertIsNotNone(dbm)
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
            robot = random.choice(list(self.robot_configs.keys()))
            # Pick a random topic
            topic_list = self.topic_configs[self.robot_configs[robot]["node-type"]]
            topic = random.choice(topic_list)
            id = du.get_topic_id_from_name(self.robot_configs, self.topic_configs,
                                           robot, topic["msg_topic"],
                                           self.test_ros_node)
            topic_find = du.get_topic_name_from_id(self.robot_configs,
                                                   self.topic_configs, robot,
                                                   id,
                                                   self.test_ros_node)
            self.assertEqual(topic["msg_topic"], topic_find)

    def test_robot_id(self):
        for robot in self.robot_configs:
            number = du.get_robot_id_from_name(self.robot_configs, robot)
            robot_name = du.get_robot_name_from_id(self.robot_configs, number)
            self.assertEqual(robot, robot_name)

if __name__ == '__main__':
    # Run test cases!
    unittest.main()
