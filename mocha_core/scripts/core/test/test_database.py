#!/usr/bin/env python3
import unittest
import sys
import os
import uuid
import geometry_msgs.msg
import rospkg
import pdb
import rospy
from colorama import Fore, Back, Style
import yaml
import pprint

ROBOT0_TOPIC0_PRIO0 = b'\x00\x00\x00u\x00\xde'
ROBOT0_TOPIC1_PRIO1_OLD = b'\x00\x01\x00v\x00\xde'
ROBOT0_TOPIC1_PRIO1_NEW = b'\x00\x01\x00w\x00\xde'
ROBOT1_TOPIC0_PRIO4_NEW = b'\x01\x00\x01O\x00\xdc'
ROBOT1_TOPIC0_PRIO4_OLD = b'\x01\x00\x01O\x00\xc7'


class test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)
        super().setUp()

    def tearDown(self):
        rospy.sleep(1)
        super().tearDown()

    def test_get_header_list(self):
        dbl = sample_db.get_sample_dbl()
        for i in range(5):
            # Check for locked mutex with loop
            # Test get all headers
            headers = dbl.get_header_list()
            self.assertTrue(ROBOT0_TOPIC0_PRIO0 in headers)
            self.assertTrue(ROBOT1_TOPIC0_PRIO4_OLD in headers)
            # The order of headers should also be respected by the
            # priority. ROBOT0_TOPIC0_PRIO0 has the lowest priority
            # among the headers so it should be the last of the list.
            # Conversely, ROBOT1_TOPIC0_PRIO4_NEW has the highest priority
            # and it should be first
            self.assertEqual(headers[-1], ROBOT0_TOPIC0_PRIO0)
            self.assertEqual(headers[0], ROBOT1_TOPIC0_PRIO4_NEW)
            # Test get headers filtering by robot
            headers_rfilt = dbl.get_header_list(filter_robot_id=0)
            self.assertTrue(ROBOT0_TOPIC0_PRIO0 in headers_rfilt)
            self.assertFalse(ROBOT1_TOPIC0_PRIO4_NEW in headers_rfilt)
            # TOPIC2 should also be the last header after filtering
            self.assertEqual(headers_rfilt[-1], ROBOT0_TOPIC0_PRIO0)
            # TOPIC 4 and 3 should be the second and third headers. Their
            # priority is 1, so they should be ordered by timestamp
            # latest timestamp
            self.assertEqual(headers_rfilt[1], ROBOT0_TOPIC1_PRIO1_NEW)
            self.assertEqual(headers_rfilt[2], ROBOT0_TOPIC1_PRIO1_OLD)
            # Test timestamp filtering. Only one timestamp should be
            # remaining (and it is not ROBOT0_TOPIC0_PRIO0).
            headers_tfilt = dbl.get_header_list(filter_robot_id=0,
                                                filter_latest=True)
            self.assertTrue(ROBOT0_TOPIC0_PRIO0 in headers_tfilt)
            self.assertTrue(ROBOT0_TOPIC1_PRIO1_NEW in headers_tfilt)
            self.assertFalse(ROBOT0_TOPIC1_PRIO1_OLD in headers_tfilt)
            self.assertFalse(ROBOT1_TOPIC0_PRIO4_OLD in headers_tfilt)
            self.assertFalse(ROBOT1_TOPIC0_PRIO4_NEW in headers_tfilt)
            self.assertTrue(len(headers_tfilt) == 2)
            headers_tfilt = dbl.get_header_list(filter_latest=True)
            self.assertTrue(ROBOT0_TOPIC0_PRIO0 in headers_tfilt)
            self.assertTrue(ROBOT0_TOPIC1_PRIO1_NEW in headers_tfilt)
            self.assertTrue(ROBOT1_TOPIC0_PRIO4_NEW in headers_tfilt)
            self.assertFalse(ROBOT0_TOPIC1_PRIO1_OLD in headers_tfilt)
            self.assertFalse(ROBOT1_TOPIC0_PRIO4_OLD in headers_tfilt)
            self.assertTrue(len(headers_tfilt) == 3)

    def test_headers_not_in_local(self):
        dbl = sample_db.get_sample_dbl()
        header_list = dbl.get_header_list()
        extra_header_1 = du.generate_random_header()
        extra_header_2 = du.generate_random_header()
        header_list.append(extra_header_2)
        header_list.append(extra_header_1)
        new_headers = [extra_header_1, extra_header_2]
        new_headers.sort()
        discover_extra_header = dbl.headers_not_in_local(header_list)
        discover_extra_header.sort()
        self.assertListEqual(discover_extra_header,
                             new_headers)

    def test_find_header(self):
        dbl = sample_db.get_sample_dbl()
        header = b'\x01\x00\x01O\x00\xc7'
        robot_id = 1
        topic_id = 0
        dtype = dbl.db[robot_id][topic_id][header].dtype
        prio = dbl.db[robot_id][topic_id][header].priority
        ts = dbl.db[robot_id][topic_id][header].ts
        data = dbl.db[robot_id][topic_id][header].data
        dbm = dbl.find_header(header)
        self.assertEqual(dbm.robot_id, robot_id)
        self.assertEqual(dbm.topic_id, topic_id)
        self.assertEqual(dbm.dtype, dtype)
        self.assertEqual(dbm.priority, prio)
        self.assertAlmostEqual(dbm.ts, ts)
        self.assertEqual(dbm.data, data)


if __name__ == '__main__':
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('mocha_core')
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import database
    import sample_db
    import hash_comm as hc
    import database_utils as du

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
