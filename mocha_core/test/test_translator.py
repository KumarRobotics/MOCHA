#!/usr/bin/env python3

import os
import pdb
import random
import sys
import time
import unittest
import warnings
import threading
from pprint import pprint

import rclpy
import rclpy.time
import rclpy.clock
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.node import Node
import yaml
from colorama import Fore, Style
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

import mocha_core.database_server as ds
import mocha_core.database_utils as du
import mocha_core.translator as tr
from mocha_core.srv import AddUpdateDB, GetDataHeaderDB, SelectDB


class Database_server_test(Node):
    def __init__(self):
        # Important to match the topic that the translator expects
        super().__init__("mocha")


class test_translator(unittest.TestCase):
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
        self.executor = MultiThreadedExecutor(num_threads=1)
        self.executor.add_node(self.test_server_node)
        executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        executor_thread.start()

        # Create a database server object with the node (this will create the services)
        self.dbs = ds.DatabaseServer(self.robot_configs, self.topic_configs,
                                     self.robot_name, self.test_server_node)

        # Create an empty list for the translator nodes
        self.test_translator_nodes = set()

        super().setUp()

    def setUpTranslator(self):
        # Use the actual TranslatorNode from translator.py with class configs
        test_translator_node = tr.TranslatorNode(
            this_robot=self.robot_name,
            robot_configs=self.robot_configs_path,
            topic_configs=self.topic_configs_path
        )
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(test_translator_node)
        logger = test_translator_node.get_logger()
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        self.test_translator_nodes.add((test_translator_node, executor, logger))
        return test_translator_node, logger, executor_thread

    def tearDown(self):
        self.executor.shutdown()
        self.test_server_node.destroy_node()
        for node, executor, _ in self.test_translator_nodes:
            executor.shutdown()
            node.destroy_node()
        rclpy.shutdown()
        time.sleep(1)
        super().tearDown()

    def test_translator_subscriptions_created(self):
        """Test that translator creates subscriptions for robot topics"""
        # Create translator node
        translator_node, logger, _ = self.setUpTranslator()

        # Wait for subscriptions to be established
        time.sleep(1.0)

        # Get the expected topics for charon robot
        this_robot_topics = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]

        # Check that translator has exactly one subscription for each topic
        for topic in this_robot_topics:
            topic_name = topic["msg_topic"]
            subscriptions_info = translator_node.get_subscriptions_info_by_topic(topic_name)
            self.assertEqual(len(subscriptions_info), 1,
                              f"Expected exactly 1 subscription for {topic_name} topic, got {len(subscriptions_info)}")

    def test_translator_processes_messages(self):
        """Test that translator processes messages and stores them in database"""
        # Create translator node
        translator_node, logger, _ = self.setUpTranslator()

        # Create a publisher on the translator node to send messages to the translator
        test_publisher = translator_node.create_publisher(
            PointStamped,
            "/pose",
            10
        )

        # Wait for connections to be established
        time.sleep(1.0)

        # Create and publish a test message
        sample_msg = PointStamped()
        sample_msg.header.frame_id = "world"
        sample_msg.point.x = random.random()
        sample_msg.point.y = random.random()
        sample_msg.point.z = random.random()
        sample_msg.header.stamp = translator_node.get_clock().now().to_msg()

        # Publish the message (translator should receive and store it)
        test_publisher.publish(sample_msg)

        # Wait for translator to process the message
        time.sleep(2.0)

        # Check that the message was stored in the database by querying the database
        try:
            robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)
            # Create request to check database
            req = SelectDB.Request()
            req.robot_id = robot_id
            req.topic_id = 0  # Not being used
            req.ts_limit = translator_node.get_clock().now().to_msg() # Not being used

            # Use the translator node's service client to query the database
            select_client = translator_node.create_client(
                SelectDB,
                "/mocha/select_db",
                qos_profile=ds.DatabaseServer.QOS_PROFILE
            )
            while not select_client.wait_for_service(timeout_sec=1.0):
                pass

            future = select_client.call_async(req)
            rclpy.spin_until_future_complete(translator_node, future)
            answ = future.result()
            returned_headers = du.deserialize_headers(answ.headers)

            # Verify exactly one message was stored
            self.assertEqual(len(returned_headers), 1,
                              f"Expected exactly 1 message in database, got {len(returned_headers)}")
            # print(f"Database headers: {returned_headers}")

        except Exception as exc:
            print(f"Database query failed: {exc}")
            self.assertTrue(False)

    def test_create_translators_for_robot(self):
        """Test that translators are created for all topics of a robot"""
        # Create translator node using TranslatorNode class
        translator_node, logger, _ = self.setUpTranslator()

        # Create translators for charon robot (should create /odometry and /pose translators)
        this_robot_topics = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]
        msg_types = du.msg_types(self.robot_configs, self.topic_configs, translator_node)

        translators = []
        for topic_id, topic in enumerate(this_robot_topics):
            # Get robot id
            robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)
            obj = msg_types[robot_id][topic_id]["obj"]
            translator = tr.Translator(
                self.robot_name, robot_id, topic["msg_topic"], topic_id, obj, translator_node
            )
            translators.append(translator)

        # Verify we created the expected number of translators
        expected_topics = len(this_robot_topics)
        self.assertEqual(len(translators), expected_topics,
                        f"Expected {expected_topics} translators, got {len(translators)}")

        # Wait for subscriptions to be established
        time.sleep(0.5)

        # Check that each topic has exactly one subscription
        for topic_id, topic in enumerate(this_robot_topics):
            topic_name = topic["msg_topic"]
            subscriptions_info = translator_node.get_subscriptions_info_by_topic(topic_name)
            self.assertEqual(len(subscriptions_info), 2,
                              f"Expected exactly 1 subscription for {topic_name} topic, got {len(subscriptions_info)}")

    def test_translator_storm(self):
        """Storm test: publish many messages to translator topics and verify they're all stored"""
        MSGS_PER_TOPIC = 20

        # Create translator node
        translator_node, logger, _ = self.setUpTranslator()

        # Wait for translator to be fully set up
        time.sleep(1.0)

        # Get available topics for this robot
        this_robot_topics = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]
        msg_types = du.msg_types(self.robot_configs, self.topic_configs, translator_node)
        robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)

        def topic_publisher(topic_id, topic_info, msgs_count):
            """Publisher function for a single topic - runs in its own thread"""
            topic_name = topic_info["msg_topic"]

            # Create a separate node for publishing
            pub_node = Node(f"topic_publisher_{topic_name.replace('/', '_')}")

            # Get message type for this topic
            obj = msg_types[robot_id][topic_id]["obj"]
            pub = pub_node.create_publisher(obj, topic_name, 10)

            # Wait for publisher to connect
            time.sleep(0.5)

            # Publish messages with 2ms delay between each
            for i in range(msgs_count):
                # Create random message based on topic type
                if topic_name == "/pose":
                    msg = PointStamped()
                    msg.header.frame_id = "world"
                    msg.point.x = random.random()
                    msg.point.y = random.random()
                    msg.point.z = random.random()
                    msg.header.stamp = pub_node.get_clock().now().to_msg()
                elif topic_name == "/odometry":
                    msg = Odometry()
                    msg.header.frame_id = "world"
                    msg.pose.pose.position.x = random.random()
                    msg.pose.pose.position.y = random.random()
                    msg.pose.pose.position.z = random.random()
                    msg.header.stamp = pub_node.get_clock().now().to_msg()
                else:
                    continue  # Skip unknown topic types

                pub.publish(msg)
                # Wait 300 ms between messages to avoid double publication
                time.sleep(0.3)

            pub_node.destroy_node()

        # Launch one thread per topic
        threads = []
        for topic_id, topic in enumerate(this_robot_topics):
            thread = threading.Thread(target=topic_publisher, args=(topic_id, topic, MSGS_PER_TOPIC))
            thread.start()
            threads.append(thread)

        # Wait for all publishing threads to complete
        for thread in threads:
            thread.join()

        # Wait for translator to process all messages
        time.sleep(1.0)

        # Query the database to count stored messages
        try:
            robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)
            req = SelectDB.Request()
            req.robot_id = robot_id
            req.topic_id = 0 # This is not used as filtering is not implemented
            req.ts_limit = translator_node.get_clock().now().to_msg() # Not used

            select_client = translator_node.create_client(
                SelectDB,
                "/mocha/select_db",
                qos_profile=ds.DatabaseServer.QOS_PROFILE
            )
            while not select_client.wait_for_service(timeout_sec=1.0):
                pass

            future = select_client.call_async(req)
            rclpy.spin_until_future_complete(translator_node, future)
            answ = future.result()
            returned_headers = du.deserialize_headers(answ.headers)

            # Calculate expected total messages
            # Each topic gets MSGS_PER_TOPIC messages
            # charon robot has 2 topics: /pose and /odometry
            expected_total = MSGS_PER_TOPIC * len(this_robot_topics)

            # Verify all messages were stored
            self.assertEqual(len(returned_headers), expected_total,
                              f"Expected {expected_total} messages in database, got {len(returned_headers)}")

        except Exception as exc:
            print(f"Database query failed: {exc}")
            self.assertTrue(False)

if __name__ == "__main__":
    unittest.main()
