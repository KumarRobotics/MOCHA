#!/usr/bin/env python3
import os
import unittest
import pdb
import time
import yaml
import rclpy
import rclpy.time
import threading
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from colorama import Fore, Style
import sample_db
import mocha_core.synchronize_channel as sync
import mocha_core.database_utils as du
import mocha_core.database as db

class Synchronize_channel_test(Node):
    def __init__(self):
        super().__init__("test_synchronize_channel")

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

    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        rclpy.init()
        self.test_ros_node = Synchronize_channel_test()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.test_ros_node)
        self.logger = self.test_ros_node.get_logger()
        executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        executor_thread.start()
        super().setUp()

    def tearDown(self):
        time.sleep(1)
        self.executor.shutdown()
        self.test_ros_node.destroy_node()
        rclpy.shutdown()
        super().tearDown()

    def test_onehop_oneway_sync(self):
        """ Start both DBs, insert a message in DB2, trigger_sync DB1, and check that both DBs are the same """
        dbl1 = sample_db.get_sample_dbl() 
        dbl2 = sample_db.get_sample_dbl()
        dbm = db.DBMessage(1, 1, 2, 1,
                           rclpy.time.Time(seconds=123, nanoseconds=456000000), bytes('New data', 'utf-8'))
        dbl2.add_modify_data(dbm)
        node_1 = sync.Channel(dbl1, 'basestation', 'charon',
                              self.robot_configs, 2, self.test_ros_node)
        node_2 = sync.Channel(dbl2, 'charon', 'basestation',
                              self.robot_configs, 2, self.test_ros_node)
        node_1.run()
        node_2.run()
        node_1.trigger_sync()
        # Wait for comms propagation
        time.sleep(2)
        node_1.stop()
        node_2.stop()
        self.assertDictEqual(dbl1.db, dbl2.db)
        # This sleep avoid getting an error because the socket is
        # already in use
        time.sleep(4)

    def test_convoluted_onehop_oneway_sync(self):
        """ Start one DB1, insert a message in DB2 but don't start, trigger_sync
        DB1 (should be busy), start DB2, trigger_sync DB1, DBs should be the same """
        self.maxDiff=None
        dbl1 = sample_db.get_sample_dbl()
        dbl2 = sample_db.get_sample_dbl()
        # print(id(db1), id(db2))
        # Fixed: using global db reference
        # Modify one of the features in the db
        dbm = db.DBMessage(1, 2, 2, 1,
                           rclpy.time.Time(seconds=123, nanoseconds=456000000), bytes('New data', 'utf-8'))
        dbl2.add_modify_data(dbm)
        node_1 = sync.Channel(dbl1, 'basestation', 'charon', self.robot_configs,
                              2, self.test_ros_node)
        node_1.run()
        node_1.trigger_sync()
        time.sleep(8)

        node_2 = sync.Channel(dbl2, 'charon', 'basestation', self.robot_configs,
                              2, self.test_ros_node)
        # Start the comm channel
        node_2.run()

        # Trigger a synchronization
        for i in range(10):
            node_2.trigger_sync()
        node_1.trigger_sync()

        # Wait for comms propagation
        time.sleep(2)
        self.assertDictEqual(dbl1.db, dbl2.db)

        # Stop the server and wait unti all is killed
        node_1.stop()
        node_2.stop()
        # This sleep avoid getting an error because the socker is
        # already in use
        time.sleep(4)

    def test_convoluted_onehop_twoway_sync(self):
        """ Insert a message in DB1, start DB1, insert a message in DB2 but don't start, trigger_sync
        DB1 (should be busy), start DB2, trigger_sync DB1, DBs should be the same """
        self.maxDiff=None
        dbl1 = sample_db.get_sample_dbl()
        dbl2 = sample_db.get_sample_dbl()
        # Modify one of the features in the db
        dbm = db.DBMessage(1, 2,  2, 1,
                           rclpy.time.Time(seconds=123, nanoseconds=456000000), bytes('New data', 'utf-8'))
        dbl1.add_modify_data(dbm)
        dbm = db.DBMessage(1, 3, 2, 1,
                           rclpy.time.Time(seconds=123, nanoseconds=456000000), bytes('New data', 'utf-8'))
        dbl2.add_modify_data(dbm)
        node_1 = sync.Channel(dbl1, 'basestation',
                              'charon', self.robot_configs, 2, self.test_ros_node)
        node_1.run()
        node_1.trigger_sync()
        time.sleep(8)

        node_2 = sync.Channel(dbl2, 'charon', 'basestation',
                              self.robot_configs, 2, self.test_ros_node)
        # Start the comm channel
        node_2.run()

        # Trigger a synchronization
        for i in range(10):
            node_2.trigger_sync()
        node_1.trigger_sync()

        # Wait for comms propagation
        time.sleep(2)
        self.assertDictEqual(dbl1.db, dbl2.db)

        # Stop the server and wait unti all is killed
        node_1.stop()
        node_2.stop()
        # This sleep avoid getting an error because the socker is
        # already in use
        time.sleep(4)

    def test_twohop_oneway_sync(self):
        dbl_robot1 = sample_db.get_sample_dbl()
        dbl_groundstation = sample_db.get_sample_dbl()
        dbl_robot2 = sample_db.get_sample_dbl()
        # Modify one of the features in the db
        dbm = db.DBMessage(1, 2, 2, 1,
                           rclpy.time.Time(seconds=123, nanoseconds=456000000), bytes('New data', 'utf-8'))
        dbl_robot1.add_modify_data(dbm)
        node_1 = sync.Channel(dbl_robot1, 'charon',
                              'basestation', self.robot_configs, 2, self.test_ros_node)
        node_2 = sync.Channel(dbl_groundstation,
                              'basestation',
                              'charon', self.robot_configs, 2, self.test_ros_node)
        node_3 = sync.Channel(dbl_groundstation,
                              'basestation',
                              'styx', self.robot_configs, 2, self.test_ros_node)
        node_4 = sync.Channel(dbl_robot2,
                              'styx', 'basestation',
                              self.robot_configs, 2, self.test_ros_node)

        node_1.run()
        node_2.run()
        node_3.run()
        node_4.run()

        # Trigger first sync
        node_2.trigger_sync()

        # Wait for comms propagation
        time.sleep(2)
        self.assertDictEqual(dbl_groundstation.db, dbl_robot1.db)

        # Trigger second sync
        node_4.trigger_sync()
        time.sleep(2)
        self.assertDictEqual(dbl_robot2.db, dbl_groundstation.db)
        self.assertDictEqual(dbl_robot2.db, dbl_robot1.db)

        # Wait until all the servers are killed
        node_1.stop()
        node_2.stop()
        node_3.stop()
        node_4.stop()
        # This sleep avoid getting an error because the socker is
        # already in use
        time.sleep(2)


    def test_delay_run(self):
        self.maxDiff = None
        db_gs = sample_db.get_sample_dbl()
        db_r1 = sample_db.get_sample_dbl()
        db_r2 = sample_db.get_sample_dbl()

        node_gs_r1 = sync.Channel(db_gs, 'basestation', 'charon', self.robot_configs, 2, self.test_ros_node)
        node_gs_r2 = sync.Channel(db_gs, 'basestation', 'styx', self.robot_configs, 2, self.test_ros_node)

        node_r1_gs = sync.Channel(db_r1, 'charon', 'basestation', self.robot_configs, 2, self.test_ros_node)
        node_r2_gs = sync.Channel(db_r2, 'styx', 'basestation', self.robot_configs, 2, self.test_ros_node)

        node_r1_r2 = sync.Channel(db_r1, 'charon', 'styx', self.robot_configs, 2, self.test_ros_node)
        node_r2_r1 = sync.Channel(db_r2, 'styx', 'charon', self.robot_configs, 2, self.test_ros_node)

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

        time.sleep(2)

    def test_multi_robot_sync(self):
        self.maxDiff=None

        db_gs = sample_db.get_sample_dbl()
        db_r1 = sample_db.get_sample_dbl()

        # Set up all channels
        node_gs_r1 = sync.Channel(db_gs, 'basestation',
                                  'charon', self.robot_configs, 2, self.test_ros_node)
        node_r1_gs = sync.Channel(db_r1, 'charon',
                                  'basestation', self.robot_configs, 2,
                                  self.test_ros_node)

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

        self.assertDictEqual(db_gs.db,db_r1.db)
        time.sleep(2)

if __name__ == '__main__':
    unittest.main()
