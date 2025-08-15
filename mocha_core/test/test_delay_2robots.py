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
from colorama import Fore, Style

# Load robot configs for ROS2
current_dir = os.path.dirname(os.path.abspath(__file__))
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
        self.node = rclpy.create_node('test_delay_2robots_node')
    
    def tearDown(self):
        # Shutdown ROS2 node
        if hasattr(self, 'node'):
            self.node.destroy_node()
        
        time.sleep(1)
        super().tearDown()

    def test_delay_run(self):
        self.maxDiff = None
        db_gs = sdb.get_sample_dbl()
        db_r1 = sdb.get_sample_dbl()
        db_r2 = sdb.get_sample_dbl()

        node_gs_r1 = sync.Channel(db_gs, 'basestation', 'charon', robot_configs, client_timeout, self.node)
        node_gs_r2 = sync.Channel(db_gs, 'basestation', 'styx', robot_configs, client_timeout, self.node)

        node_r1_gs = sync.Channel(db_r1, 'charon', 'basestation', robot_configs, client_timeout, self.node)
        node_r2_gs = sync.Channel(db_r2, 'styx', 'basestation', robot_configs, client_timeout, self.node)

        node_r1_r2 = sync.Channel(db_r1, 'charon', 'styx', robot_configs, client_timeout, self.node)
        node_r2_r1 = sync.Channel(db_r2, 'styx', 'charon', robot_configs, client_timeout, self.node)

        pprint(db_gs.db)
        pprint(db_r1.db)
        pprint(db_r2.db)

        node_gs_r1.run()
        node_gs_r2.run()
        
        dbm = db.DBMessage(1, 1, 2, 1, rclpy.time.Time(seconds=int(time.time())), bytes('r1_data', 'utf-8'))
        db_r1.add_modify_data(dbm)

        node_gs_r1.trigger_sync()
        time.sleep(4)

        node_r1_gs.run()
        node_r2_gs.run()

        dbm = db.DBMessage(2, 2, 1, 2, rclpy.time.Time(seconds=int(time.time())), bytes('r2_data', 'utf-8'))
        db_r2.add_modify_data(dbm)
        
        node_r1_r2.run()

        node_r1_r2.trigger_sync()
        time.sleep(4)

        node_gs_r2.trigger_sync()
        time.sleep(2)

        node_r2_r1.run()

        node_gs_r1.stop()
        node_r1_gs.stop()

        node_r1_r2.stop()
        node_r2_r1.stop()

        node_r2_gs.stop()
        node_gs_r2.stop()

        pprint(db_gs.db)
        pprint(db_r1.db)
        pprint(db_r2.db)
        time.sleep(2)

    def test_multiple_trigger_sync(self):

        self.maxDiff = None
        db_gs = sdb.get_sample_dbl()
        db_r1 = sdb.get_sample_dbl()
        db_r2 = sdb.get_sample_dbl()

        node_gs_r1 = sync.Channel(db_gs, 'basestation', 'charon', robot_configs, client_timeout, self.node)
        node_gs_r2 = sync.Channel(db_gs, 'basestation', 'styx', robot_configs, client_timeout, self.node)

        node_r1_gs = sync.Channel(db_r1, 'charon', 'basestation', robot_configs, client_timeout, self.node)
        node_r2_gs = sync.Channel(db_r2, 'styx', 'basestation', robot_configs, client_timeout, self.node)

        node_r1_r2 = sync.Channel(db_r1, 'charon', 'styx', robot_configs, client_timeout, self.node)
        node_r2_r1 = sync.Channel(db_r2, 'styx', 'charon', robot_configs, client_timeout, self.node)

#        pprint(db_gs)
#        pprint(db_r1)
#        pprint(db_r2)

#        node_r1_r2.run()
        node_r2_r1.run()

        dbm = db.DBMessage(1, 1, 2, 1, rclpy.time.Time(seconds=int(time.time())), bytes('R1_data', 'utf-8'))
        db_r1.add_modify_data(dbm)

        node_r2_r1.trigger_sync()
        time.sleep(5)

#        node_r2_r1.trigger_sync()
#        time.sleep(5)

#        node_r1_r2.stop()
        node_r2_r1.stop()
        time.sleep(2)

if __name__ == '__main__':
    unittest.main()
