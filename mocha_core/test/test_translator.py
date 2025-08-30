#!/usr/bin/env python3

import os
import pdb
import random
import sys
import time
import unittest
import threading

import rclpy
import rclpy.time
import rclpy.clock
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml
from colorama import Fore, Style
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

import mocha_core.database_server as ds
import mocha_core.database_utils as du
import mocha_core.translator as tr
from mocha_core.srv import SelectDB


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
        self.executor_server = MultiThreadedExecutor(num_threads=4)
        self.executor_server.add_node(self.test_server_node)
        executor_server_thread = threading.Thread(target=self.executor_server.spin, daemon=True)
        executor_server_thread.start()

        # Create a database server object with the node (this will create the services)
        self.dbs = ds.DatabaseServer(self.robot_configs, self.topic_configs,
                                     self.robot_name, self.test_server_node)

        # Use the actual TranslatorNode from translator.py with class configs
        self.test_translator_node = tr.TranslatorNode(
            this_robot=self.robot_name,
            robot_configs=self.robot_configs_path,
            topic_configs=self.topic_configs_path
        )
        self.executor_translator = MultiThreadedExecutor(num_threads=4)
        self.executor_translator.add_node(self.test_translator_node)
        executor_translator_thread = threading.Thread(target=self.executor_translator.spin, daemon=True)
        executor_translator_thread.start()
        time.sleep(1) # Let things talk to each other
        super().setUp()


    def tearDown(self):
        self.executor_server.shutdown()
        self.test_server_node.destroy_node()
        self.executor_translator.shutdown()
        self.test_translator_node.destroy_node()
        rclpy.shutdown()
        time.sleep(1)
        super().tearDown()

    def test_translator_subscriptions_created(self):
        """Test that translator creates subscriptions for robot topics"""
        # Get the expected topics for charon robot
        this_robot_topics = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]

        # Check that translator has exactly one subscription for each topic
        for topic in this_robot_topics:
            topic_name = topic["msg_topic"]
            subscriptions_info = self.test_translator_node.get_subscriptions_info_by_topic(topic_name)
            self.assertEqual(len(subscriptions_info), 1,
                              f"Expected exactly 1 subscription for {topic_name} topic, got {len(subscriptions_info)}")

    def test_translator_storm(self):
        """Storm test: publish many messages to translator topics and verify they're all stored"""
        MSGS_PER_TOPIC = 10

        # Get available topics for this robot
        this_robot_topics = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]
        msg_types = du.msg_types(self.robot_configs, self.topic_configs,
                                 self.test_translator_node)
        robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)

        # Create a node that does the pubisher thing. We will instantiate this
        # in multiple different threads

        class FakePublisher(Node):
            def __init__(self, topic_id, topic_info, msg_types):
                random_name = f"fake_publiser_{random.randint(0, 2**32)}"
                super().__init__(random_name)

                topic_name = topic_info["msg_topic"]

                # Get message type for this topic
                obj = msg_types[robot_id][topic_id]["obj"]
                self.pub = self.create_publisher(obj, topic_name, tr.QOS_PROFILE)

                # Block until we are ready to go
                self.mx = threading.Lock()

                # Wait for publisher to connect
                time.sleep(1)

                self.mx.acquire()
                for i in range(MSGS_PER_TOPIC):
                    # Create random message based on topic type
                    if topic_name == "/pose":
                        msg = PointStamped()
                        msg.header.frame_id = "world"
                        msg.point.x = random.random()
                        msg.point.y = random.random()
                        msg.point.z = random.random()
                        msg.header.stamp = self.get_clock().now().to_msg()
                    elif topic_name == "/odometry":
                        msg = Odometry()
                        msg.header.frame_id = "world"
                        msg.pose.pose.position.x = random.random()
                        msg.pose.pose.position.y = random.random()
                        msg.pose.pose.position.z = random.random()
                        msg.header.stamp = self.get_clock().now().to_msg()
                    self.pub.publish(msg)
                    time.sleep(.2)


        # Launch one thread per topic
        executors = []
        nodes = []
        for topic_id, topic in enumerate(this_robot_topics):
            pub_node = FakePublisher(topic_id, topic, msg_types)
            nodes.append(pub_node)
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(pub_node)
            executors.append(executor)
            thread = threading.Thread(target=executor.spin)
            thread.start()
        # Wait until things are stable
        time.sleep(1)

        # Go!
        for node in nodes:
            node.mx.release()
        time.sleep(1)

        # Wait for all publishing threads to complete
        for executor in executors:
            executor.shutdown()

        for node in nodes:
            node.destroy_node()

        # Query the database to count stored messages
        try:
            robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)
            req = SelectDB.Request()
            req.robot_id = robot_id
            req.topic_id = 0 # This is not used as filtering is not implemented
            req.ts_limit = self.test_translator_node.get_clock().now().to_msg() # Not used

            select_client = self.test_translator_node.create_client(
                SelectDB,
                "/mocha/select_db",
                qos_profile=ds.DatabaseServer.QOS_PROFILE
            )
            while not select_client.wait_for_service(timeout_sec=1.0):
                pass

            future = select_client.call_async(req)
            rclpy.spin_until_future_complete(self.test_translator_node, future)
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
