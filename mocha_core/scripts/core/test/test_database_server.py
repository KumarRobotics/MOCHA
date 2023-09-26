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
import mocha_core.srv
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
        rospy.wait_for_service('~GetDataHeaderDB')
        rospy.wait_for_service('~SelectDB')

        # Generate the service calls for all the services
        service_name = '~AddUpdateDB'
        self.add_update_db = rospy.ServiceProxy(service_name,
                                                mocha_core.srv.AddUpdateDB,
                                                persistent=True)

        service_name = '~GetDataHeaderDB'
        self.get_data_header_db = rospy.ServiceProxy(service_name,
                                                     mocha_core.srv.GetDataHeaderDB,
                                                     persistent=True)

        service_name = '~SelectDB'
        self.select_db = rospy.ServiceProxy(service_name,
                                            mocha_core.srv.SelectDB,
                                            persistent=True)

        super().setUp()

    def tearDown(self):
        self.add_update_db.close()
        self.get_data_header_db.close()
        self.select_db.close()
        self.dbs.shutdown()
        rospy.sleep(1)
        super().tearDown()

    def test_add_retrieve_single_msg(self):
        # Simulate sending a "/pose" message from Charon
        sample_msg = PointStamped()
        sample_msg.header.frame_id = "world"
        sample_msg.point.x = random.random()
        sample_msg.point.y = random.random()
        sample_msg.point.z = random.random()
        sample_msg.header.stamp = rospy.Time.now()

        tid = du.get_topic_id_from_name(robot_configs, topic_configs,
                                        robot_name, "/pose")
        # Serialize and send through service
        serialized_msg = du.serialize_ros_msg(sample_msg)

        try:
            answ = self.add_update_db(tid,
                                      rospy.get_rostime(),
                                      serialized_msg)
            answ_header = answ.new_header
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        # print("Got hash:", answ_hash)
        # The data is now stored in the database!

        # Request the same hash through service
        try:
            answ = self.get_data_header_db(answ_header)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)

        # Parse answer and compare results
        _, ans_topic_id, _, ans_data = du.parse_answer(answ,
                                                       msg_types)

        self.assertEqual(tid, ans_topic_id)
        self.assertEqual(ans_data, sample_msg)
        # print("Received topic:", ans_topic_name)
        # print("ROS msg:", ans_data)
        # print("Timestamp:", ans_ts)

    def test_add_select_robot(self):
        stored_headers = []
        for i in range(3):
            # Sleep is very important to have distinct messages
            rospy.sleep(0.1)
            # Create a dumb random PointStamped message
            sample_msg = PointStamped()
            sample_msg.header.frame_id = "world"
            sample_msg.point.x = random.random()
            sample_msg.point.y = random.random()
            sample_msg.point.z = random.random()
            sample_msg.header.stamp = rospy.Time.now()
            tid = du.get_topic_id_from_name(robot_configs, topic_configs,
                                            robot_name, "/pose")

            # Serialize and send through service
            serialized_msg = du.serialize_ros_msg(sample_msg)
            try:
                answ = self.add_update_db(tid,
                                          rospy.get_rostime(),
                                          serialized_msg)
                answ_header = answ.new_header
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                self.assertTrue(False)
            stored_headers.append(answ_header)

        # Request the list of headers through the service
        try:
            robot_id = du.get_robot_id_from_name(robot_configs, "charon")
            answ = self.select_db(robot_id, None, None)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        returned_headers = du.deserialize_headers(answ.headers)

        # Sort list before comparing
        stored_headers.sort()
        returned_headers.sort()
        self.assertEqual(stored_headers, returned_headers)

    def test_insert_storm(self):
        # Bomb the database with insert petitions from different robots,
        # check the number of headers afterwards
        NUM_PROCESSES = 100
        LOOPS_PER_PROCESS = 10

        # Spin a number of processes to write into the database
        def recorder_thread():
            # Get a random ros time
            tid = du.get_topic_id_from_name(robot_configs, topic_configs,
                                            robot_name, "/pose")
            sample_msg = PointStamped()
            sample_msg.header.frame_id = "world"
            sample_msg.point.x = random.random()
            sample_msg.point.y = random.random()
            sample_msg.point.z = random.random()
            sample_msg.header.stamp = rospy.Time.now()
            # Serialize and send through service
            serialized_msg = du.serialize_ros_msg(sample_msg)

            for i in range(LOOPS_PER_PROCESS):
                timestamp = rospy.Time(random.randint(1, 10000),
                                       random.randint(0, 1000)*1000000)
                try:
                    _ = self.add_update_db(tid,
                                           timestamp,
                                           serialized_msg)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
                    self.assertTrue(False)

        process_list = []
        for i in range(NUM_PROCESSES):
            x = multiprocessing.Process(target=recorder_thread)
            process_list.append(x)

        for p in process_list:
            p.start()

        for p in process_list:
            p.join()

        # Get the list of hashes from the DB and count them
        try:
            robot_id = du.get_robot_id_from_name(robot_configs, "charon")
            answ = self.select_db(robot_id, None, None)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            self.assertTrue(False)
        returned_headers = du.deserialize_headers(answ.headers)

        self.assertEqual(len(returned_headers),
                         NUM_PROCESSES*LOOPS_PER_PROCESS)


if __name__ == '__main__':
    rospy.init_node('test_database_server', anonymous=False)

    # Get the directory path and import all the required modules to test
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('mocha_core')
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

    msg_types = du.msg_types(robot_configs, topic_configs)

    # Set the ~robot_name param to charon
    robot_name = "charon"
    rospy.set_param("~robot_name", robot_name)

    # Run test cases
    unittest.main()
