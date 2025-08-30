#!/usr/bin/env python3

import os
import pdb
import time
import unittest
import threading

import rclpy
from rclpy.node import Node
import yaml
from colorama import Fore, Style
import mocha_core.database_server as ds
import mocha_core.topic_publisher as tp
from mocha_core.database import DBMessage
from rclpy.executors import MultiThreadedExecutor
import geometry_msgs.msg

class Database_server_test(Node):
    def __init__(self):
        super().__init__("mocha")


class test_topic_publisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load configurations at the class level
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ddb_path = os.path.join(current_dir, "..")

        # Load robot configs
        cls.robot_configs_path = os.path.join(ddb_path, "config/testConfigs/robot_configs.yaml")
        with open(cls.robot_configs_path, "r") as f:
            cls.robot_configs = yaml.load(f, Loader=yaml.FullLoader)

        # Load topic configs
        cls.topic_configs_path = os.path.join(ddb_path, "config/testConfigs/topic_configs.yaml")
        with open(cls.topic_configs_path, "r") as f:
            cls.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        cls.robot_name = "charon"

    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20 * "=", test_name, 20 * "=", Style.RESET_ALL)
        rclpy.init()
        self.test_server_node = Database_server_test()
        self.executor_db = MultiThreadedExecutor(num_threads=4)
        self.executor_db.add_node(self.test_server_node)
        executor_thread_db = threading.Thread(target=self.executor_db.spin, daemon=True)
        executor_thread_db.start()

        # Create a database server object with the node (this will create the services)
        self.dbs = ds.DatabaseServer(self.robot_configs, self.topic_configs,
                                     self.robot_name, self.test_server_node)

        # Create the topic_puiblisher node
        self.test_topic_publisher_node = tp.TopicPublisherNode(self.robot_name,
                                                               self.robot_configs_path,
                                                               self.topic_configs_path)
        self.executor_tp = MultiThreadedExecutor(num_threads=4)
        self.executor_tp.add_node(self.test_topic_publisher_node)
        self.logger = self.test_topic_publisher_node.get_logger()
        executor_thread_tp = threading.Thread(target=self.executor_tp.spin, daemon=True)
        executor_thread_tp.start()
        time.sleep(1) # Wait for nodes to be set up
        super().setUp()

    def tearDown(self):
        self.test_topic_publisher_node.shutdown()
        self.executor_db.shutdown()
        self.test_server_node.destroy_node()
        self.executor_tp.shutdown()
        self.test_topic_publisher_node.destroy_node()
        rclpy.shutdown()
        time.sleep(1) # Wait final teardown
        super().tearDown()

    def test_topic_publisher_creates_topics(self):
        """ Verify that all the topics created are there with the corresponding
        namespace """

        # Get all topics that have publishers
        topic_names_and_types = self.test_topic_publisher_node.get_topic_names_and_types()

        expected_names_and_types = {'/basestation/target_goals':
                                     'geometry_msgs/msg/PoseArray',
                                    '/charon/odometry':
                                    'nav_msgs/msg/Odometry',
                                    '/charon/pose':
                                    'geometry_msgs/msg/PointStamped',
                                    '/mocha/database_cb':
                                    'mocha_core/msg/DatabaseCB',
                                    '/quad1/image':
                                    'sensor_msgs/msg/Image',
                                    '/quad1/pose':
                                    'geometry_msgs/msg/PointStamped',
                                    '/styx/odometry':
                                    'nav_msgs/msg/Odometry',
                                    '/styx/pose':
                                    'geometry_msgs/msg/PointStamped'}

        # Verify that the topics are there
        for name, topictype in topic_names_and_types:
            # Skip generic topics
            if name == "/parameter_events" or name == "/rosout":
                continue
            self.assertIn(name, expected_names_and_types.keys())
            self.assertEqual(topictype[0], expected_names_and_types[name])

    def test_publisher_publish_msg(self):
        """ Verify that when a message is added to the database we will get this
            message published by the publisher """

        # Create a new topic to listen to the publisher
        class ListenerNode(Node):
            def __init__(self):
                super().__init__("listener_node")
                self.subscriptor = self.create_subscription(
                    geometry_msgs.msg.PointStamped,
                    '/charon/pose',
                    self.topic_cb,
                    qos_profile=tp.QOS_PROFILE
                )
                self.result = None
                self.counter = 0

            def topic_cb(self, data):
                self.result = data
                self.counter += 1

        test_listener_node = ListenerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(test_listener_node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        time.sleep(1)
        # Insert a message known to be from charon, pose
        bin_data = b'\x04"M\x18h@,\x00\x00\x00\x00\x00\x00\x00<\x12\x00\x00\x005\x00\x01\x00\x01\x00\x07\x0b\x00\x0c\x02\x00P\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        msg_count = 10
        for i in range(msg_count):
            msg = DBMessage(1, 1, 0, 0,
                            rclpy.time.Time(nanoseconds=1083902000000 + i*1000000),
                            bin_data)
            # Insert the message in the database in a crude way
            self.dbs.dbl.add_modify_data(msg)
        # Let the publishers publish
        time.sleep(3)
        executor.shutdown()
        test_listener_node.destroy_node()
        time.sleep(1)
        # We should have msg_count messages on the reader
        self.assertEqual(msg_count, test_listener_node.counter)
        self.assertIsInstance(test_listener_node.result,
                              geometry_msgs.msg._point_stamped.PointStamped)


if __name__ == "__main__":
    unittest.main(failfast=True)
