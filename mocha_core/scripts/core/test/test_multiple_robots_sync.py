#!/usr/bin/env python3
import unittest
import sys
import uuid
import time
from pprint import pprint
import multiprocessing
sys.path.append('..')
import get_sample_db as sdb
import synchronize_channel as sync
import synchronize_utils as su
import database_server_utils as du
from colorama import Fore, Back, Style

CONFIG_FILE = "testConfigs/robotConfigs.yml"

class test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)

     def test_multi_robot_sync(self):
        self.maxDiff=None

        db_gs = sdb.get_sample_dbl()
        db_r1 = sdb.get_sample_dbl()

        # Set up all channels
        node_gs_r1 = sync.Channel(db_gs, 'groundstation',
                                  'charon', CONFIG_FILE)
        node_r1_gs = sync.Channel(db_r1, 'charon',
                                  'groundstation', CONFIG_FILE)

        pprint(db_gs.db)
        pprint(db_r1.db)

        node_r1_gs.run()
        node_gs_r1.run()

        nodes_1 = 3
        robot_prefix = 'R1_'
        feature_prefix = 'Node_'
        for i in range(nodes_1):
            feature = robot_prefix + feature_prefix + str(i)
            dbm = su.DBMessage(1, feature, 0, 1, time.time(),
                    bytes('r1_data', 'utf-8'), False)
            du.add_modify_data_dbl(db_r1, dbm)

        node_gs_r1.trigger_sync()
        time.sleep(2)
        node_r1_gs.trigger_sync()
        time.sleep(2)
        
        dbm = su.DBMessage(0, 'Node_0', 1, 0, time.time(),
                bytes('r1_data', 'utf-8'), False)
        du.add_modify_data_dbl(db_gs, dbm)

        node_r1_gs.trigger_sync()
        time.sleep(2)
        node_gs_r1.trigger_sync()
        time.sleep(2)

        node_r1_gs.stop()
        node_gs_r1.stop()

        pprint(db_gs.db)
        pprint(db_r1.db)

        self.assertDictEqual(db_gs.db,db_r1.db)
        time.sleep(2)

if __name__ == '__main__':
    unittest.main()
