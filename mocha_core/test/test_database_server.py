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

import rclpy
import rclpy.time
import rclpy.clock
import yaml
from colorama import Fore, Style
from geometry_msgs.msg import PointStamped

import mocha_core.srv


class test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20 * "=", test_name, 20 * "=", Style.RESET_ALL)

        # Ignore pesky warnings about sockets
        warnings.filterwarnings(
            action="ignore", message="unclosed", category=ResourceWarning
        )

        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Create a ROS2 node for the database server
        self.node = rclpy.create_node('test_database_server_node')
        
        # Create a database server object with the node (this will create the services)
        self.dbs = ds.DatabaseServer(robot_configs, topic_configs, node=self.node)

        super().setUp()

    def tearDown(self):
        self.dbs.shutdown()
        
        # Shutdown ROS2 node and spin thread
        if hasattr(self, 'node'):
            self.node.destroy_node()
        
        import time
        time.sleep(1)
        super().tearDown()

    def test_add_retrieve_single_msg(self):
        # Simulate sending a "/pose" message from Charon
        sample_msg = PointStamped()
        sample_msg.header.frame_id = "world"
        sample_msg.point.x = random.random()
        sample_msg.point.y = random.random()
        sample_msg.point.z = random.random()
        sample_msg.header.stamp = rclpy.clock.Clock().now().to_msg()

        tid = du.get_topic_id_from_name(
            robot_configs, topic_configs, robot_name, "/pose"
        )
        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)

        try:
            # Create request and call service method directly
            req = mocha_core.srv.AddUpdateDB.Request()
            req.topic_id = tid
            req.timestamp = rclpy.clock.Clock().now().to_msg()
            req.msg_content = serialized_msg
            answ = self.dbs.add_update_db_service_cb(req)
            answ_header = answ.new_header
        except Exception as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        # print("Got hash:", answ_hash)
        # The data is now stored in the database!

        # Request the same hash through service
        try:
            # Create request and call service method directly
            req = mocha_core.srv.GetDataHeaderDB.Request()
            req.msg_header = answ_header
            answ = self.dbs.get_data_hash_db_service_cb(req)
        except Exception as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Parse answer and compare results
        _, ans_topic_id, _, ans_data = du.parse_answer(answ, msg_types)

        self.assertEqual(tid, ans_topic_id)
        self.assertEqual(ans_data, sample_msg)
        # print("Received topic:", ans_topic_name)
        # print("ROS msg:", ans_data)
        # print("Timestamp:", ans_ts)

    def test_add_select_robot(self):
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
                robot_configs, topic_configs, robot_name, "/pose"
            )

            # Serialize and send through service
            serialized_msg = du.serialize_ros_msg(sample_msg)
            try:
                # Create request and call service method directly
                req = mocha_core.srv.AddUpdateDB.Request()
                req.topic_id = tid
                req.timestamp = rclpy.clock.Clock().now().to_msg()
                req.msg_content = serialized_msg
                answ = self.dbs.add_update_db_service_cb(req)
                answ_header = answ.new_header
            except Exception as exc:
                print("Service did not process request: " + str(exc))
                self.assertTrue(False)
            stored_headers.append(answ_header)

        # Request the list of headers through the service
        try:
            robot_id = du.get_robot_id_from_name(robot_configs, "charon")
            # Create request and call service method directly
            req = mocha_core.srv.SelectDB.Request()
            req.robot_id = robot_id
            req.topic_id = 0  # Changed from None to 0 since it's uint8
            req.ts_limit = rclpy.clock.Clock().now().to_msg()
            answ = self.dbs.select_db_service_cb(req)
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
        def recorder_thread():
            # Get a random ros time
            tid = du.get_topic_id_from_name(
                robot_configs, topic_configs, robot_name, "/pose"
            )
            sample_msg = PointStamped()
            sample_msg.header.frame_id = "world"
            sample_msg.point.x = random.random()
            sample_msg.point.y = random.random()
            sample_msg.point.z = random.random()
            sample_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            # Serialize and send through service
            serialized_msg = du.serialize_ros_msg(sample_msg)

            for i in range(LOOPS_PER_THREAD):
                timestamp = rclpy.time.Time(
                    seconds=random.randint(1, 10000),
                    nanoseconds=random.randint(0, 1000) * 1000000,
                ).to_msg()
                try:
                    # Create request and call service method directly
                    req = mocha_core.srv.AddUpdateDB.Request()
                    req.topic_id = tid
                    req.timestamp = timestamp
                    req.msg_content = serialized_msg
                    _ = self.dbs.add_update_db_service_cb(req)
                except Exception as exc:
                    print(f"Service did not process request: {exc}")

        import threading
        thread_list = []
        for i in range(NUM_THREADS):
            x = threading.Thread(target=recorder_thread)
            thread_list.append(x)

        for t in thread_list:
            t.start()

        for t in thread_list:
            t.join()

        # Get the list of hashes from the DB and count them
        try:
            robot_id = du.get_robot_id_from_name(robot_configs, "charon")
            # Create request and call service method directly
            req = mocha_core.srv.SelectDB.Request()
            req.robot_id = robot_id
            req.topic_id = 0  # Changed from None to 0 since it's uint8
            req.ts_limit = rclpy.clock.Clock().now().to_msg()
            answ = self.dbs.select_db_service_cb(req)
        except Exception as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        returned_headers = du.deserialize_headers(answ.headers)

        self.assertEqual(len(returned_headers), NUM_THREADS * LOOPS_PER_THREAD)


# Add the mocha_core module path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
mocha_core_path = os.path.join(current_dir, "..", "mocha_core")
sys.path.append(mocha_core_path)

import database_server as ds
import database_utils as du

# Load robot configs
robot_configs_path = os.path.join(
    current_dir, "..", "config", "testConfigs", "robot_configs.yaml"
)
with open(robot_configs_path, "r") as f:
    robot_configs = yaml.load(f, Loader=yaml.FullLoader)

# Load topic configs
topic_configs_path = os.path.join(
    current_dir, "..", "config", "testConfigs", "topic_configs.yaml"
)
with open(topic_configs_path, "r") as f:
    topic_configs = yaml.load(f, Loader=yaml.FullLoader)

msg_types = du.msg_types(robot_configs, topic_configs)

# Set robot name
robot_name = "charon"

if __name__ == "__main__":
    unittest.main()
