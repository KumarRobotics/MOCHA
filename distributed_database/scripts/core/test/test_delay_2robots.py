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
from colorama import Fore, Style

CONFIG_FILE = "testConfigs/robotConfigs.yml"

class test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)

    def test_delay_run(self):
        self.maxDiff = None
        db_gs = sdb.get_sample_dbl()
        db_r1 = sdb.get_sample_dbl()
        db_r2 = sdb.get_sample_dbl()

        node_gs_r1 = sync.Channel(db_gs, 'groundstation', 'charon', CONFIG_FILE)
        node_gs_r2 = sync.Channel(db_gs, 'groundstation', 'styx', CONFIG_FILE)

        node_r1_gs = sync.Channel(db_r1, 'charon', 'groundstation', CONFIG_FILE)
        node_r2_gs = sync.Channel(db_r2, 'styx', 'groundstation', CONFIG_FILE)

        node_r1_r2 = sync.Channel(db_r1, 'charon', 'styx', CONFIG_FILE)
        node_r2_r1 = sync.Channel(db_r2, 'styx', 'charon', CONFIG_FILE)

        pprint(db_gs.db)
        pprint(db_r1.db)
        pprint(db_r2.db)

        node_gs_r1.run()
        node_gs_r2.run()
        
        dbm = su.DBMessage(1, 'Node_1', 1, 2, time.time(), bytes('r1_data', 'utf-8'), False)
        du.add_modify_data_dbl(db_r1, dbm)

        node_gs_r1.trigger_sync()
        time.sleep(4)

        node_r1_gs.run()
        node_r2_gs.run()

        dbm = su.DBMessage(2, 'Node_2', 2, 1, time.time(), bytes('r2_data', 'utf-8'), False)
        du.add_modify_data_dbl(db_r2, dbm)
        
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

        node_gs_r1 = sync.Channel(db_gs, 'groundstation', 'charon', CONFIG_FILE)
        node_gs_r2 = sync.Channel(db_gs, 'groundstation', 'styx', CONFIG_FILE)

        node_r1_gs = sync.Channel(db_r1, 'charon', 'groundstation', CONFIG_FILE)
        node_r2_gs = sync.Channel(db_r2, 'styx', 'groundstation', CONFIG_FILE)

        node_r1_r2 = sync.Channel(db_r1, 'charon', 'styx', CONFIG_FILE)
        node_r2_r1 = sync.Channel(db_r2, 'styx', 'charon', CONFIG_FILE)

#        pprint(db_gs)
#        pprint(db_r1)
#        pprint(db_r2)

#        node_r1_r2.run()
        node_r2_r1.run()

        dbm = su.DBMessage(1, 'Node_1', 1, 2, time.time(), bytes('R1_data', 'utf-8'), False)
        du.add_modify_data_dbl(db_r1, dbm)

        node_r2_r1.trigger_sync()
        time.sleep(5)

#        node_r2_r1.trigger_sync()
#        time.sleep(5)

#        node_r1_r2.stop()
        node_r2_r1.stop()
        time.sleep(2)

if __name__ == '__main__':
    unittest.main()
