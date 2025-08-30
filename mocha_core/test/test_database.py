#!/usr/bin/env python3
import unittest
import sys
import os
import pdb
from colorama import Fore, Back, Style
import mocha_core.database as database
import sample_db
import mocha_core.hash_comm as hc
import mocha_core.database_utils as du
import copy

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
        import time
        time.sleep(1)
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
        # Test with all headers
        extra_header_1 = du.generate_random_header()
        extra_header_2 = du.generate_random_header()
        header_list.append(extra_header_2)
        header_list.append(extra_header_1)
        new_headers = [extra_header_1, extra_header_2]
        new_headers.sort()
        # Test without newer
        discover_extra_header = dbl.headers_not_in_local(header_list)
        discover_extra_header.sort()
        self.assertListEqual(discover_extra_header,
                             new_headers)
        # Test with older header, create a copy of one of the existing headers in the
        # db. Pick one with the msec field that can accomodate both addition and
        # substraction
        test_header = None
        for header in header_list:
            h = hc.TsHeader.from_header(header)
            if h.msecs > 0 and h.msecs < 999:
                test_header = header
                break
        self.assertTrue(test_header is not None)
        # Modify the timestamp to make it before the current msg
        h = hc.TsHeader.from_header(test_header)
        h.msecs -= 1
        extra_header_3 = h.bindigest()
        header_list.append(extra_header_3)
        discover_extra_header_latest = dbl.headers_not_in_local(header_list, newer=True)
        discover_extra_header_latest.sort()
        discover_extra_header_all = dbl.headers_not_in_local(header_list)
        discover_extra_header_all.sort()
        self.assertListEqual(discover_extra_header_latest, new_headers)
        diff_header = set(discover_extra_header_all) - set(discover_extra_header)
        self.assertEqual(diff_header.pop(), extra_header_3)
        # Modify the timestamp to make it after the current msg
        h = hc.TsHeader.from_header(test_header)
        h.msecs += 1
        extra_header_4 = h.bindigest()
        header_list.append(extra_header_4)
        discover_extra_header_latest = dbl.headers_not_in_local(header_list, newer=True)
        new_headers.append(extra_header_4) # The new header should show in the list
        new_headers.sort()
        discover_extra_header_latest.sort()
        self.assertListEqual(new_headers, discover_extra_header_latest)
        # Finally, getting all the headers should not filter them out
        discover_extra_header = dbl.headers_not_in_local(header_list)
        self.assertTrue(len(discover_extra_header) == 4)

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
    # Run test cases!
    unittest.main()
