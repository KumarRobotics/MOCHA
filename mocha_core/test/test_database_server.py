#!/usr/bin/env python3

import multiprocessing
import os
import pdb
import random
import sys
import time
import unittest
import warnings
from pprint import pprint
import threading
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import rclpy
import rclpy.time
import rclpy.clock
from rclpy.logging import LoggingSeverity
import yaml
from colorama import Fore, Style
from geometry_msgs.msg import PointStamped
import mocha_core.database_server as ds
import mocha_core.database_utils as du
from mocha_core.srv import AddUpdateDB, GetDataHeaderDB, SelectDB


import mocha_core.srv

class Database_server_test(Node):
    def __init__(self):
        super().__init__("test_database_server")


class Database_client_test(Node):
    def __init__(self):
        name_rnd = random.randint(0, 2**32)
        self.node_name = f"test_database_client_{name_rnd}"
        super().__init__(self.node_name)
        logger = self.get_logger()
        logger.set_level(LoggingSeverity.DEBUG)
        self.add_update_db_cli = self.create_client(AddUpdateDB,
                                                    "/test_database_server/add_update_db")
        while not self.add_update_db_cli.wait_for_service(timeout_sec=1.0):
            logger.debug("Waiting for add_update_db")
        self.get_data_header_db_cli = self.create_client(GetDataHeaderDB,
                                                    "/test_database_server/get_data_header_db")
        while not self.get_data_header_db_cli.wait_for_service(timeout_sec=1.0):
            logger.debug("Waiting for get_data_header_db")
        self.select_db_cli = self.create_client(SelectDB,
                                                    "/test_database_server/select_db")
        while not self.select_db_cli.wait_for_service(timeout_sec=1.0):
            logger.debug("Waiting for get_data_header_db")

    def recorder(self, loops_per_thread, robot_configs, topic_configs, robot_name):
        # Get a random ros time
        tid = du.get_topic_id_from_name(
            robot_configs, topic_configs, robot_name,
            "/pose",
            self
        )
        sample_msg = PointStamped()
        sample_msg.header.frame_id = "world"
        sample_msg.point.x = random.random()
        sample_msg.point.y = random.random()
        sample_msg.point.z = random.random()
        sample_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)

        def response_callback(future):
            pass

        for i in range(loops_per_thread):
            timestamp = rclpy.time.Time(
                seconds=random.randint(0, 65535),
                nanoseconds=random.randint(0, 1000) * 1000000,
            ).to_msg()
            try:
                # Create request and call service method directly
                req = mocha_core.srv.AddUpdateDB.Request()
                req.topic_id = tid
                req.timestamp = timestamp
                req.msg_content = serialized_msg
                future = self.add_update_db_cli.call_async(req)
                future.add_done_callback(response_callback)
                # rclpy.spin_until_future_complete(self, future)
            except Exception as exc:
                print(f"Service did not process request: {exc}")


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
        cls.robot_name = "charon"


    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20 * "=", test_name, 20 * "=", Style.RESET_ALL)

        rclpy.init()
        self.test_server_node = Database_server_test()
        ds.DatabaseServer(self.robot_configs, self.topic_configs, self.robot_name,
                          self.test_server_node)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.test_server_node)
        executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        executor_thread.start()

        # Create an empty list for the client nodes
        self.test_client_nodes = set()

        # Create a database server object with the node (this will create the services)
        self.dbs = ds.DatabaseServer(self.robot_configs, self.topic_configs,
                                     self.robot_name, self.test_server_node)
        super().setUp()

    def setUpClient(self):
        test_client_node = Database_client_test()
        executor = SingleThreadedExecutor()
        executor.add_node(test_client_node)
        logger = test_client_node.get_logger()
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        self.test_client_nodes.add((test_client_node, executor, logger))
        return test_client_node, logger

    def tearDown(self):
        self.executor.shutdown()
        self.test_server_node.destroy_node()
        for node, executor, _ in self.test_client_nodes:
            executor.shutdown()
            node.destroy_node()
        rclpy.shutdown()
        time.sleep(1)
        super().tearDown()

    def test_add_retrieve_single_msg(self):
        # Create a single client
        client_node, _ = self.setUpClient()

        # Simulate sending a "/pose" message from Charon
        sample_msg = PointStamped()
        sample_msg.header.frame_id = "world"
        sample_msg.point.x = random.random()
        sample_msg.point.y = random.random()
        sample_msg.point.z = random.random()
        sample_msg.header.stamp = rclpy.clock.Clock().now().to_msg()

        tid = du.get_topic_id_from_name(
            self.robot_configs, self.topic_configs, 
            self.robot_name, "/pose",
            client_node
        )
        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)

        try:
            # Create request and call service method directly
            req = mocha_core.srv.AddUpdateDB.Request()
            req.topic_id = tid
            req.timestamp = rclpy.clock.Clock().now().to_msg()
            req.msg_content = serialized_msg
            future = client_node.add_update_db_cli.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            answ = future.result()
            answ_header = answ.new_header
        except Exception as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        # print("Got hash:", answ_header)
        # The data is now stored in the database!

        # Request the same hash through service
        try:
            # Create request and call service method directly
            req = mocha_core.srv.GetDataHeaderDB.Request()
            req.msg_header = answ_header
            future = client_node.get_data_header_db_cli.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            answ = future.result()
        except Exception as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Parse answer and compare results
        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs,
                             client_node)
        _, ans_topic_id, _, ans_data = du.parse_answer(answ, self.msg_types)

        self.assertEqual(tid, ans_topic_id)
        self.assertEqual(ans_data, sample_msg)
        # print("Received topic:", ans_topic_name)
        # print("ROS msg:", ans_data)
        # print("Timestamp:", ans_ts)

    def test_add_select_robot(self):
        # Create a single client
        client_node, _ = self.setUpClient()

        stored_headers = []
        for i in range(3):
            # Sleep is very important to have distinct messages
            time.sleep(0.1)
            # Create a dumb random PointStamped message
            sample_msg = PointStamped()
            sample_msg.header.frame_id = "world"
            sample_msg.point.x = random.random()
            sample_msg.point.y = random.random()
            sample_msg.point.z = random.random()
            sample_msg.header.stamp = rclpy.clock.Clock().now().to_msg()

            tid = du.get_topic_id_from_name(
                self.robot_configs, self.topic_configs, self.robot_name,
                "/pose",
                client_node
            )

            # Serialize and send through service
            serialized_msg = du.serialize_ros_msg(sample_msg)
            try:
                # Create request and call service method directly
                req = mocha_core.srv.AddUpdateDB.Request()
                req.topic_id = tid
                req.timestamp = rclpy.clock.Clock().now().to_msg()
                req.msg_content = serialized_msg
                future = client_node.add_update_db_cli.call_async(req)
                rclpy.spin_until_future_complete(client_node, future)
                answ = future.result()
                answ_header = answ.new_header
            except Exception as exc:
                print("Service did not process request: " + str(exc))
                self.assertTrue(False)
            stored_headers.append(answ_header)

        # Request the list of headers through the service
        try:
            robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)
            # Create request and call service method directly
            req = mocha_core.srv.SelectDB.Request()
            req.robot_id = robot_id
            req.topic_id = 0  # Changed from None to 0 since it's uint8
            req.ts_limit = rclpy.clock.Clock().now().to_msg()
            future = client_node.select_db_cli.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            answ = future.result()
        except Exception as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        returned_headers = du.deserialize_headers(answ.headers)

        # Sort list before comparing
        stored_headers.sort()
        returned_headers.sort()
        self.assertEqual(stored_headers, returned_headers)

    def test_insert_storm(self):
        # Bomb the database with insert petitions from different robots,
        # check the number of headers afterwards
        # Note: Using threading instead of multiprocessing to test concurrent access
        # since ROS2 objects don't serialize well across processes
        NUM_THREADS = 100
        LOOPS_PER_THREAD = 10

        # Spin a number of threads to write into the database (tests concurrent access)

        import threading
        thread_list = []
        for i in range(NUM_THREADS):
            client_node, _ = self.setUpClient()
            client_node.recorder(LOOPS_PER_THREAD, self.robot_configs,
                                 self.topic_configs, self.robot_name)

        for t in thread_list:
            t.start()

        for t in thread_list:
            t.join()

        # Get the list of hashes from the DB and count them
        try:
            robot_id = du.get_robot_id_from_name(self.robot_configs, self.robot_name)
            # Create request and call service method directly
            req = mocha_core.srv.SelectDB.Request()
            req.robot_id = robot_id
            req.topic_id = 0  # Changed from None to 0 since it's uint8
            req.ts_limit = rclpy.clock.Clock().now().to_msg()
            response = mocha_core.srv.SelectDB.Response()
            answ = self.dbs.select_db_service_cb(req, response)
        except Exception as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        returned_headers = du.deserialize_headers(answ.headers)

        self.assertEqual(len(returned_headers), NUM_THREADS * LOOPS_PER_THREAD)


if __name__ == "__main__":
    unittest.main()
