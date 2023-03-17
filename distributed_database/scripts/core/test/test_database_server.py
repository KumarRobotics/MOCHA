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
from geometry_msgs.msg import PointStamped
import yaml
from colorama import Fore, Style
import random
import warnings


class Test(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20*"=", test_name, 20*"=", Style.RESET_ALL)

        # Ignore pesky warnings about sockets
        warnings.filterwarnings(action="ignore", message="unclosed", category=ResourceWarning)

        # Create a database server object
        self.dbs = ds.DatabaseServer(robot_configs, topic_configs)

        rospy.wait_for_service('~AddUpdateDB')
        rospy.wait_for_service('~GetDataHashDB')
        rospy.wait_for_service('~SelectDB')
        rospy.wait_for_service('~SetAckBitDB')

        # Generate the service calls for all the services
        service_name = '~AddUpdateDB'
        self.add_update_db = rospy.ServiceProxy(service_name,
                                                distributed_database.srv.AddUpdateDB,
                                                persistent=True)

        service_name = '~GetDataHashDB'
        self.get_data_hash_db = rospy.ServiceProxy(service_name,
                                                   distributed_database.srv.GetDataHashDB,
                                                   persistent=True)

        service_name = '~SelectDB'
        self.select_db = rospy.ServiceProxy(service_name,
                                            distributed_database.srv.SelectDB,
                                            persistent=True)

        service_name = '~SetAckBitDB'
        self.toggle_ack_bit_db = rospy.ServiceProxy(service_name,
                                                    distributed_database.srv.SetAckBitDB,
                                                    persistent=True)

        super().setUp()

    def tearDown(self):
        self.add_update_db.close()
        self.get_data_hash_db.close()
        self.select_db.close()
        self.toggle_ack_bit_db.close()
        self.dbs.shutdown()
        rospy.sleep(1)
        super().tearDown()

    def test_add_retrieve_single_msg(self):
        # Create a dumb random PointStamped message
        sample_msg = PointStamped()
        sample_msg.header.frame_id = "world"
        sample_msg.point.x = random.random()
        sample_msg.point.y = random.random()
        sample_msg.point.z = random.random()
        sample_msg.header.stamp = rospy.Time.now()
        topic_name = 'TopicX'

        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)
        try:
            answ = self.add_update_db(topic_name,
                                      sample_msg._md5sum,
                                      0,
                                      serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        # print("Got hash:", answ_hash)
        # The data is now stored in the database!

        # Request the same hash through service
        try:
            # Keep in mind that hashes need to be str!
            answ = self.get_data_hash_db(answ_hash)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Parse answer and compare results
        ans_topic_name, ans_ts, ans_data, _ = du.parse_answer(answ,
                                                              self.dbs.msg_types)

        self.assertEqual(topic_name, ans_topic_name)
        self.assertEqual(ans_data, sample_msg)
        # print("Received topic:", ans_topic_name)
        # print("ROS msg:", ans_data)
        # print("Timestamp:", ans_ts)

    def test_add_select_robot(self):
        stored_hashes = []
        for i in range(3):
            # Create a dumb random PointStamped message
            sample_msg = PointStamped()
            sample_msg.header.frame_id = "world"
            sample_msg.point.x = random.random()
            sample_msg.point.y = random.random()
            sample_msg.point.z = random.random()
            sample_msg.header.stamp = rospy.Time.now()
            topic_name = 'topic_name' + str(i)

            # Serialize and send through service
            serialized_msg = du.serialize_ros_msg(sample_msg)
            try:
                answ = self.add_update_db(topic_name,
                                          sample_msg._md5sum,
                                          0,
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
        # print("Stored hashes:", stored_hashes)
        # print("Returned hashes:", returned_hashes)
        self.assertEqual(stored_hashes, returned_hashes)

    def test_insert_storm(self):
        # Bomb the database with insert petitions, check the number of
        # hashes afterwards
        NUM_PROCESSES = 10
        LOOPS_PER_PROCESS = 20

        # Spin a number of processes to write into the database
        def recorder_thread(topic_name):
            for i in range(LOOPS_PER_PROCESS):
                sample_msg = PointStamped()
                sample_msg.header.frame_id = "world"
                sample_msg.point.x = random.random()
                sample_msg.point.y = random.random()
                sample_msg.point.z = random.random()
                sample_msg.header.stamp = rospy.Time.now()
                topic_name = topic_name + str(i)

                # Serialize and send through service
                serialized_msg = du.serialize_ros_msg(sample_msg)
                try:
                    answ = self.add_update_db(topic_name,
                                              sample_msg._md5sum,
                                              0,
                                              serialized_msg)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
                    self.assertTrue(False)

        process_list = []
        for i in range(NUM_PROCESSES):
            topic_name = "topic" + str(i) + "-"
            x = multiprocessing.Process(target=recorder_thread,
                                        args=(topic_name,))
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
        sample_msg = PointStamped()
        sample_msg.header.frame_id = "world"
        sample_msg.point.x = random.random()
        sample_msg.point.y = random.random()
        sample_msg.point.z = random.random()
        sample_msg.header.stamp = rospy.Time.now()
        topic_name = 'topic_name'

        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)
        try:
            answ = self.add_update_db(topic_name,
                                      sample_msg._md5sum,
                                      0,
                                      serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Request the same hash through service
        try:
            answ = self.get_data_hash_db(answ_hash)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Check that the Ack bit is effectively false
        _, _, _, ans_ack = du.parse_answer(answ, self.dbs.msg_types)
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
            answ = self.get_data_hash_db(new_answ_hash)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Check that the Ack bit is effectively True
        _, _, _, ans_ack = du.parse_answer(answ, self.dbs.msg_types)
        self.assertTrue(ans_ack)


if __name__ == '__main__':
    rospy.init_node('test_database_server', anonymous=False)

    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import database_utils as du
    import database_server as ds

    # Get the default path from the ddb_path
    robot_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/robot_configs.yaml")
    # Get the path to the robot config file from the ros parameter robot_configs
    robot_configs = rospy.get_param("robot_configs",
                                    robot_configs_default)

    # Get the yaml dictionary objects
    with open(robot_configs, "r") as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Get the default path from the ddb_path
    topic_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/topic_configs.yaml")
    # Get the path to the robot config file from the ros parameter topic_configs
    topic_configs = rospy.get_param("topic_configs",
                                    topic_configs_default)

    # Get the yaml dictionary objects
    with open(topic_configs, "r") as f:
        topic_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Set the ~robot_name param to charon
    rospy.set_param("~robot_name", "charon")

    # Run test cases
    unittest.main()
