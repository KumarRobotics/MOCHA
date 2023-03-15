#!/usr/bin/env python3
import os
import unittest
import sys
import uuid
import pdb
import time
import yaml
import rospkg
import rospy
import multiprocessing
from pprint import pprint
from colorama import Fore, Style


class Test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)

    def test_onehop_oneway_sync(self):
        dbl1 = sdb.get_sample_dbl()
        dbl2 = sdb.get_sample_dbl()
        dbm = su.DBMessage(1, 'fetureX', 2, 1,
                           123.456, bytes('New data', 'utf-8'), False)
        du.add_modify_data_dbl(dbl2, dbm)
        node_1 = sync.Channel(dbl1, 'basestation',
                              'charon', robot_configs)
        node_2 = sync.Channel(dbl2, 'charon',
                              'basestation', robot_configs)
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
        self.maxDiff=None
        dbl1 = sdb.get_sample_dbl()
        dbl2 = sdb.get_sample_dbl()
        # print(id(db1), id(db2))
        # Modify one of the features in the db
        dbm = su.DBMessage(1, 'fetureX', 2, 1,
                           123.456, bytes('New data', 'utf-8'), False)
        du.add_modify_data_dbl(dbl2, dbm)
        node_1 = sync.Channel(dbl1, 'basestation',
                              'charon', robot_configs)
        node_1.run()
        node_1.trigger_sync()
        time.sleep(8)

        node_2 = sync.Channel(dbl2, 'charon', 'basestation',
                              robot_configs)
        # Start the comm channel
        node_2.run()

        # Trigger a synchronization
        for i in range(10):
            try:
                node_2.trigger_sync()
            except Exception as e:
                print("Exception")
                pass
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
        self.maxDiff=None
        dbl1 = sdb.get_sample_dbl()
        dbl2 = sdb.get_sample_dbl()
        # Modify one of the features in the db
        dbm = su.DBMessage(1, 'fetureX', 2, 1,
                           123.456, bytes('New data', 'utf-8'), False)
        du.add_modify_data_dbl(dbl1, dbm)
        dbm = su.DBMessage(1, 'fetureY', 2, 1,
                           123.456, bytes('New data', 'utf-8'), False)
        du.add_modify_data_dbl(dbl2, dbm)
        node_1 = sync.Channel(dbl1, 'basestation',
                              'charon', robot_configs)
        node_1.run()
        node_1.trigger_sync()
        time.sleep(8)

        node_2 = sync.Channel(dbl2, 'charon', 'basestation',
                              robot_configs)
        # Start the comm channel
        node_2.run()

        # Trigger a synchronization
        for i in range(10):
            try:
                node_2.trigger_sync()
            except Exception as e:
                print("Exception")
                pass
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
        dbl_robot1 = sdb.get_sample_dbl()
        dbl_groundstation = sdb.get_sample_dbl()
        dbl_robot2 = sdb.get_sample_dbl()
        # Modify one of the features in the db
        dbm = su.DBMessage(1, 'fetureX', 2, 1,
                           123.456, bytes('New data', 'utf-8'), False)
        du.add_modify_data_dbl(dbl_robot1, dbm)
        node_1 = sync.Channel(dbl_robot1, 'charon',
                              'basestation', robot_configs)
        node_2 = sync.Channel(dbl_groundstation,
                              'basestation',
                              'charon', robot_configs)
        node_3 = sync.Channel(dbl_groundstation,
                              'basestation',
                              'styx', robot_configs)
        node_4 = sync.Channel(dbl_robot2,
                              'styx', 'basestation',
                              robot_configs)

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
        time.sleep(4)

if __name__ == '__main__':
    rospy.init_node('test_synchronize_channel', anonymous=False,
                    log_level=rospy.DEBUG)

    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import get_sample_db as sdb
    import synchronize_channel as sync
    import database_server_utils as du
    import synchronize_utils as su

    # Get the default path from the ddb_path
    robot_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/robot_configs.yaml")
    # Get the path to the robot config file from the ros parameter robot_configs
    robot_configs = rospy.get_param("robot_configs",
                                    robot_configs_default)

    # Get the yaml dictionary objects
    with open(robot_configs, "r") as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Run test cases
    unittest.main()
