#!/usr/bin/env python3
import unittest
import sys
import uuid
import time
from pprint import pprint
import multiprocessing
import os
import rclpy
import rclpy.time
import yaml

# Add the mocha_core module path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
mocha_core_path = os.path.join(current_dir, "..", "mocha_core")
sys.path.append(mocha_core_path)

import sample_db as sdb
import mocha_core.synchronize_channel as sync
import mocha_core.database as db
import mocha_core.database_utils as du
from colorama import Fore, Back, Style

# Load robot configs for ROS2
robot_configs_path = os.path.join(
    current_dir, "..", "config", "testConfigs", "robot_configs.yaml"
)
with open(robot_configs_path, "r") as f:
    robot_configs = yaml.load(f, Loader=yaml.FullLoader)

client_timeout = 6.0

class test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Create a ROS2 node for testing
        self.node = rclpy.create_node('test_multiple_robots_sync_node')
    
    def tearDown(self):
        # Shutdown ROS2 node
        if hasattr(self, 'node'):
            self.node.destroy_node()
        
        time.sleep(1)
        super().tearDown()

    def test_multi_robot_sync(self):
        self.maxDiff=None

        db_gs = sdb.get_sample_dbl()
        db_r1 = sdb.get_sample_dbl()

        # Set up all channels
        node_gs_r1 = sync.Channel(db_gs, 'basestation',
                                  'charon', robot_configs, client_timeout, self.node)
        node_r1_gs = sync.Channel(db_r1, 'charon',
                                  'basestation', robot_configs, client_timeout, self.node)

        pprint(db_gs.db)
        pprint(db_r1.db)

        node_r1_gs.run()
        node_gs_r1.run()

        nodes_1 = 3
        robot_prefix = 'R1_'
        feature_prefix = 'Node_'
        for i in range(nodes_1):
            feature = robot_prefix + feature_prefix + str(i)
            current_time = time.time()
            dbm = db.DBMessage(1, 0, 1, 1, rclpy.time.Time(seconds=int(current_time), nanoseconds=int((current_time % 1) * 1e9)),
                    bytes('r1_data', 'utf-8'))
            db_r1.add_modify_data(dbm)

        node_gs_r1.trigger_sync()
        time.sleep(2)
        node_r1_gs.trigger_sync()
        time.sleep(2)
        
        current_time = time.time()
        dbm = db.DBMessage(0, 1, 0, 0, rclpy.time.Time(seconds=int(current_time), nanoseconds=int((current_time % 1) * 1e9)),
                bytes('r1_data', 'utf-8'))
        db_gs.add_modify_data(dbm)

        node_r1_gs.trigger_sync()
        time.sleep(2)
        node_gs_r1.trigger_sync()
        time.sleep(2)

        node_r1_gs.stop()
        node_gs_r1.stop()

        pprint(db_gs.db)
        pprint(db_r1.db)

        # Compare database structures by checking keys and counts
        # Since objects are different instances, we compare structure
        self.assertEqual(set(db_gs.db.keys()), set(db_r1.db.keys()))
        
        for robot_id in db_gs.db:
            self.assertEqual(set(db_gs.db[robot_id].keys()), 
                            set(db_r1.db[robot_id].keys()))
            for topic_id in db_gs.db[robot_id]:
                self.assertEqual(set(db_gs.db[robot_id][topic_id].keys()),
                                set(db_r1.db[robot_id][topic_id].keys()))
                # Verify same number of messages in each topic
                self.assertEqual(len(db_gs.db[robot_id][topic_id]),
                                len(db_r1.db[robot_id][topic_id]))
        time.sleep(2)

if __name__ == '__main__':
    unittest.main()
