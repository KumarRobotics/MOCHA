#!/usr/bin/env python3

import os
import unittest
import sys
import multiprocessing
import roslaunch
import rospy
import rospkg
import pdb
from pprint import pprint
import distributed_database.srv
import distributed_database.msg

CONFIG_FILE = "testConfigs/robotConfigs.yml"

package = 'distributed_database'
executable = 'database_server.py'


class Test(unittest.TestCase):
    def setUp(self):
        # startNode and stopNode are used to start the database_server
        # node during the test case. *Do not* start the database_server
        # yourself in your code!
        node = roslaunch.core.Node(package, executable)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self.process = launch.launch(node)

        # Generate the callbacks for all the services
        service_name = 'database_server/AddUpdateDB'
        rospy.wait_for_service(service_name)
        self.add_update_db = rospy.ServiceProxy(service_name,
                             distributed_database.srv.AddUpdateDB)

        service_name = 'database_server/GetDataHashDB'
        rospy.wait_for_service(service_name)
        self.get_data_hash_db = rospy.ServiceProxy(service_name,
                                distributed_database.srv.GetDataHashDB)

        service_name = 'database_server/SelectDB'
        rospy.wait_for_service(service_name)
        self.select_db = rospy.ServiceProxy(service_name,
                         distributed_database.srv.SelectDB)

        service_name = 'database_server/SetAckBitDB'
        rospy.wait_for_service(service_name)
        self.toggle_ack_bit_db = rospy.ServiceProxy(service_name,
                                 distributed_database.srv.SetAckBitDB)


    def tearDown(self):
        self.process.stop()

    def test_add_retrieve_single_msg(self):
        # Create a dumb distributed_database msg
        sample_msg = distributed_database.msg.RobotState()
        sample_msg.state.data = 3
        feature_name = 'feat_name'

        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)
        try:
            answ = self.add_update_db(feature_name,
                                      sample_msg._md5sum,
                                      serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        print("Got hash:", answ_hash)
        # The data is now stored in the database!

        # Request the same hash through service
        try:
            # Keep in mind that hashes need to be str!
            answ = self.get_data_hash_db(answ_hash)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Parse answer and compare results
        ans_feat_name, ans_ts, ans_data, _ = du.parse_answer(answ)

        self.assertEqual(feature_name, ans_feat_name)
        self.assertEqual(ans_data, sample_msg)
        print("Received feature:", ans_feat_name)
        print("ROS msg:", ans_data)
        print("Timestamp:", ans_ts)

    def test_add_select_robot(self):
        # Create three dumb messages and put them into the database.
        stored_hashes = []
        for i in range(3):
            sample_msg = distributed_database.msg.RobotState()
            sample_msg.state.data = 3
            feature_name = 'feat_name' + str(i)

            # Serialize and send through service
            serialized_msg = du.serialize_ros_msg(sample_msg)
            try:
                answ = self.add_update_db(feature_name,
                                          sample_msg._md5sum,
                                          serialized_msg)
                answ_hash = answ.new_hash
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                self.assertTrue(False)
            stored_hashes.append(answ_hash)

        # Request the list of hashes through the service
        try:
            # Keep in mind that hashes need to be str!
            answ = self.select_db("charon", -1, -1)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        returned_hashes = answ.hashes

        # Sort list before comparing
        stored_hashes.sort()
        returned_hashes.sort()

        # Parse answer and compare results
        print("Stored hashes:", stored_hashes)
        print("Returned hashes:", returned_hashes)
        self.assertEqual(stored_hashes, returned_hashes)

    def test_insert_storm(self):
        # Bomb the database with insert petitions, check the number of
        # hashes afterwards
        NUM_PROCESSES = 10
        LOOPS_PER_PROCESS = 20

        # Spin a number of processes to write into the database
        def recorder_thread(feat_name):
            for i in range(LOOPS_PER_PROCESS):
                # Create a dumb distributed_database msg
                sample_msg = distributed_database.msg.RobotState()
                sample_msg.state.data = 0
                feature_name = feat_name + str(i)

                # Serialize and send through service
                serialized_msg = du.serialize_ros_msg(sample_msg)
                try:
                    answ = self.add_update_db(feature_name,
                                              sample_msg._md5sum,
                                              serialized_msg)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
                    self.assertTrue(False)

        process_list = []
        for i in range(NUM_PROCESSES):
            feat_name = "feat" + str(i) + "-"
            x = multiprocessing.Process(target=recorder_thread,
                                        args=(feat_name,))
            process_list.append(x)

        for p in process_list:
            p.start()

        for p in process_list:
            p.join()

        # Get the list of hashes from the DB and count them
        try:
            answ = self.select_db("charon", -1, -1)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        returned_hashes = answ.hashes

        self.assertEqual(len(returned_hashes),
                         NUM_PROCESSES*LOOPS_PER_PROCESS)

    def test_toggle_ack_bit(self):
        # Put a dumb msg in the db
        sample_msg = distributed_database.msg.RobotState()
        sample_msg.state.data = 3
        feature_name = 'feat_name'

        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)
        try:
            answ = self.add_update_db(feature_name,
                                      sample_msg._md5sum,
                                      serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Request the same hash through service
        service_name = 'database_server/GetDataHashDB'
        rospy.wait_for_service(service_name)
        get_data_hash_db = rospy.ServiceProxy(service_name,
                            distributed_database.srv.GetDataHashDB)
        try:
            answ = get_data_hash_db(answ_hash)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Check that the Ack bit is effectively false
        _, _, _, ans_ack = du.parse_answer(answ)
        self.assertFalse(ans_ack)

        # Toggle the Ack Bit. This is what the basestation will do to
        # confirm the reception of the msg
        try:
            answ = self.toggle_ack_bit_db(answ_hash)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Changing the Ack bit modifies the hash, and a new hash is
        # provided
        new_answ_hash = answ.new_hash

        try:
            answ = get_data_hash_db(new_answ_hash)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Check that the Ack bit is effectively True
        _, _, _, ans_ack = du.parse_answer(answ)
        self.assertTrue(ans_ack)

    def test_get_hash_db_failure(self):
        # Request the same hash through service
        service_name = 'database_server/GetDataHashDB'
        rospy.wait_for_service(service_name)
        get_data_hash_db = rospy.ServiceProxy(service_name,
                            distributed_database.srv.GetDataHashDB)
        # We should trigger an exception because the hash is not found
        with self.assertRaises(rospy.ServiceException):
            get_data_hash_db('dadsadsa')

    def test_get_hash_db_empty(self):
        # Request the same hash through service
        service_name = 'database_server/GetDataHashDB'
        rospy.wait_for_service(service_name)
        get_data_hash_db = rospy.ServiceProxy(service_name,
                            distributed_database.srv.GetDataHashDB)
        # We should trigger an exception because the hash is not found
        with self.assertRaises(rospy.ServiceException):
            get_data_hash_db('')


if __name__ == '__main__':
    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(pkg_path, "scripts/core")
    sys.path.append(scripts_path)
    import database_server_utils as du

    # Run test cases
    suite = unittest.TestLoader().loadTestsFromTestCase(Test)
    unittest.TextTestRunner(verbosity=2).run(suite)
