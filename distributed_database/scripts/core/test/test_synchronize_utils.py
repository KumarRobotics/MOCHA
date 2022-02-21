#!/usr/bin/env python3
import unittest
import sys
import os
from pprint import pprint
import uuid
import geometry_msgs.msg
import rospkg
import pdb

VALID_HASH_FEATURE_0 = 'e7eb85cdc2d3'
VALID_HASH_FEATURE_1 = 'f50a4da37e2e'


def generate_random_hash():
    rand = str(uuid.uuid4().hex).encode()
    return hc.Hash(rand).digest()


class test(unittest.TestCase):
    def test_get_hash_list_from_db(self):
        dbl = sdb.get_sample_dbl()
        for _ in range(5):
            # Check for locked mutex with loop
            # Test get all hashes
            hashes = su.get_hash_list_from_dbl(dbl)
            self.assertTrue(VALID_HASH_FEATURE_0 in hashes)
            self.assertTrue(VALID_HASH_FEATURE_1 in hashes)
            # The order of hashes should also be respected by the
            # priority. VALID_HASH_FEATURE_0 has the lowest priority
            # among the hashes so it should be the last of the list.
            # Conversely, VALID_HASH_FEATURE_1 has the highest priority
            # and it should be first
            self.assertEqual(hashes[0], VALID_HASH_FEATURE_1)
            self.assertEqual(hashes[-1], VALID_HASH_FEATURE_0)
            # Test get hashes filtering by robot
            hashes_rfilt = su.get_hash_list_from_dbl(dbl, filter_robot=0)
            self.assertTrue(VALID_HASH_FEATURE_0 in hashes_rfilt)
            self.assertFalse(VALID_HASH_FEATURE_1 in hashes_rfilt)
            # Test for assertion when giving a robot without a ts
            with self.assertRaises(Exception):
                _ = su.get_hash_list_from_dbl(dbl, filter_ts=0.0)
            # Test timestamp filtering. Only one timestamp should be
            # remaining (and it is not VALID_HASH_FEATURE_0).
            hashes_tfilt = su.get_hash_list_from_dbl(dbl, filter_robot=0,
                                                     filter_ts=118)
            self.assertFalse(VALID_HASH_FEATURE_0 in hashes_tfilt)
            self.assertTrue(len(hashes_tfilt) == 1)

    def test_serialize_deserialize(self):
        hash_list = []
        for _ in range(20):
            hash_list.append(generate_random_hash())
        serialized = su.serialize_hashes(hash_list)
        deserialized = su.deserialize_hashes(serialized)
        self.assertEqual(deserialized, hash_list)

    def test_deserialize_wrong(self):
        hash_list = []
        for _ in range(5):
            hash_list.append(generate_random_hash())
        serialized = su.serialize_hashes(hash_list)
        with self.assertRaises(Exception):
            su.deserialize_hashes(serialized[1:])

    def test_hashes_not_in_local(self):
        dbl = sdb.get_sample_dbl()
        hash_list = su.get_hash_list_from_dbl(dbl)
        extra_hash_1 = generate_random_hash()
        extra_hash_2 = generate_random_hash()
        hash_list.append(extra_hash_2)
        hash_list.append(extra_hash_1)
        new_hashes = [extra_hash_1, extra_hash_2]
        new_hashes.sort()
        discover_extra_hash = su.hashes_not_in_local(dbl, hash_list)
        discover_extra_hash.sort()
        self.assertListEqual(discover_extra_hash,
                             new_hashes)

    def test_find_hash_db_feature(self):
        dbl = sdb.get_sample_dbl()
        robot = 1
        feature = 'feature1'
        dtype = dbl.db[robot][feature]['dtype']
        prio = dbl.db[robot][feature]['priority']
        ts = dbl.db[robot][feature]['ts']
        data = dbl.db[robot][feature]['data']
        h = dbl.db[robot][feature]['hash']
        dbm = su.find_hash_dbl(dbl, h)
        self.assertEqual(dbm.robot, robot)
        self.assertEqual(dbm.feature_name, feature)
        self.assertEqual(dbm.dtype, dtype)
        self.assertEqual(dbm.priority, prio)
        self.assertAlmostEqual(dbm.ts, ts)
        self.assertEqual(dbm.data, data)

    def test_pack_unpack_data_feature(self):
        dbl = sdb.get_sample_dbl()
        dbm = su.find_hash_dbl(dbl, VALID_HASH_FEATURE_1)
        packed = su.pack_data(dbm)
        # print(packed)
        u_dbm = su.unpack_data(packed)
        self.assertEqual(dbm, u_dbm)

    def test_verify_checksum(self):
        dbl = sdb.get_sample_dbl()
        dbm = su.find_hash_dbl(dbl, VALID_HASH_FEATURE_1)
        packed = su.pack_data(dbm)
        msg = VALID_HASH_FEATURE_1.encode() + packed
        self.assertTrue(su.verify_checksum_msg(msg))


if __name__ == '__main__':
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(pkg_path, "scripts")
    sys.path.append(scripts_path)
    import synchronize_utils as su
    import get_sample_db as sdb
    import hash_comm as hc

    # Run test cases!
    unittest.main()
