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

ROBOT0_TOPIC2_PRIO0 = '42c9f16fe4c0'
ROBOT1_TOPIC1_PRIO4 = 'd9efcff693b5'

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

    def test_serialize_deserialize(self):
        hash_list = []
        for _ in range(10):
            for _ in range(20):
                hash_list.append(generate_random_hash())
            serialized = du.serialize_hashes(hash_list)
            deserialized = du.deserialize_hashes(serialized)
            self.assertEqual(deserialized, hash_list)

    def test_deserialize_wrong(self):
        hash_list = []
        for _ in range(5):
            hash_list.append(generate_random_hash())
        serialized = du.serialize_hashes(hash_list)
        with self.assertRaises(Exception):
            du.deserialize_hashes(serialized[1:])

    def test_pack_unpack_data_topic(self):
        dbl = sample_db.get_sample_dbl()
        dbm = dbl.find_hash(ROBOT1_TOPIC1_PRIO4)
        packed = du.pack_data(dbm)
        # print(packed)
        u_dbm = du.unpack_data(packed)
        self.assertEqual(dbm, u_dbm)

    def test_verify_checksum(self):
        dbl = sample_db.get_sample_dbl()
        dbm = dbl.find_hash(ROBOT1_TOPIC1_PRIO4)
        packed = du.pack_data(dbm)
        msg = ROBOT1_TOPIC1_PRIO4.encode() + packed
        self.assertTrue(du.verify_checksum_msg(msg))


if __name__ == '__main__':
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import database_utils as du
    import sample_db
    import hash_comm as hc

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
