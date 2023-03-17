#!/usr/bin/env python3
import unittest
import sys
import os
from pprint import pprint
import uuid
import geometry_msgs.msg
import rospkg
import pdb
import rospy
from colorama import Fore, Back, Style
import yaml

VALID_HASH_FEATURE_0 = '42c9f16fe4c0'
VALID_HASH_FEATURE_1 = 'd9efcff693b5'


def generate_random_hash():
    rand = str(uuid.uuid4().hex).encode()
    return hc.Hash(rand).digest()


class test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        super().setUp()

    def tearDown(self):
        rospy.sleep(1)
        super().tearDown()

    def test_get_hash_list(self):
        dbl = sample_db.get_sample_dbl()
        for i in range(5):
            # Check for locked mutex with loop
            # Test get all hashes
            hashes = dbl.get_hash_list()
            self.assertTrue(VALID_HASH_FEATURE_0 in hashes)
            self.assertTrue(VALID_HASH_FEATURE_1 in hashes)
            # The order of hashes should also be respected by the
            # priority. VALID_HASH_FEATURE_0 has the lowest priority
            # among the hashes so it should be the last of the list.
            # Conversely, VALID_HASH_FEATURE_1 has the highest priority
            # and it should be first
            self.assertEqual(hashes[0], VALID_HASH_FEATURE_0)
            self.assertEqual(hashes[-1], VALID_HASH_FEATURE_1)
            # Test get hashes filtering by robot
            hashes_rfilt = dbl.get_hash_list(filter_robot=0)
            self.assertTrue(VALID_HASH_FEATURE_0 in hashes_rfilt)
            self.assertFalse(VALID_HASH_FEATURE_1 in hashes_rfilt)
            # Test for assertion when giving a robot without a ts
            with self.assertRaises(Exception):
                _ = dbl.get_hash_list(filter_ts=0.0)
            # Test timestamp filtering. Only one timestamp should be
            # remaining (and it is not VALID_HASH_FEATURE_0).
            hashes_tfilt = dbl.get_hash_list(filter_robot=0,
                                             filter_ts=118)
            self.assertFalse(VALID_HASH_FEATURE_0 in hashes_tfilt)
            self.assertTrue(len(hashes_tfilt) == 2)

    def test_hashes_not_in_local(self):
        dbl = sample_db.get_sample_dbl()
        hash_list = dbl.get_hash_list()
        extra_hash_1 = generate_random_hash()
        extra_hash_2 = generate_random_hash()
        hash_list.append(extra_hash_2)
        hash_list.append(extra_hash_1)
        new_hashes = [extra_hash_1, extra_hash_2]
        new_hashes.sort()
        discover_extra_hash = dbl.hashes_not_in_local(hash_list)
        discover_extra_hash.sort()
        self.assertListEqual(discover_extra_hash,
                             new_hashes)

    def test_find_hash(self):
        dbl = sample_db.get_sample_dbl()
        robot = 1
        topic = 'topic1'
        dtype = dbl.db[robot][topic]['dtype']
        prio = dbl.db[robot][topic]['priority']
        ts = dbl.db[robot][topic]['ts']
        data = dbl.db[robot][topic]['data']
        h = dbl.db[robot][topic]['hash']
        dbm = dbl.find_hash(h)
        self.assertEqual(dbm.robot, robot)
        self.assertEqual(dbm.topic_name, topic)
        self.assertEqual(dbm.dtype, dtype)
        self.assertEqual(dbm.priority, prio)
        self.assertAlmostEqual(dbm.ts, ts)
        self.assertEqual(dbm.data, data)


if __name__ == '__main__':
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import database
    import sample_db
    import hash_comm as hc

    dbl = sample_db.get_sample_dbl()

    # Set the node name
    rospy.init_node('test_synchronize_utils', anonymous=False)

    # Get the default path from the ddb_path
    robot_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/robot_configs.yaml")
    # Get the path to the robot config file from the ros parameter robot_configs
    robot_configs = rospy.get_param("robot_configs",
                                    robot_configs_default)

    # Get the yaml dictionary objects
    with open(robot_configs, "r") as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Run test cases!
    unittest.main()
