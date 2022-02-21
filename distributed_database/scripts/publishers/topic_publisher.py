import os
import pdb
import sys

import rospkg
import rospy
import yaml
import re

import distributed_database.srv
import nav_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

class TopicPublisher():
    def __init__(self, target):
        # Imports
        rospack = rospkg.RosPack()
        comms_path = rospack.get_path("distributed_database")
        scripts_path = os.path.join(comms_path, "scripts/core")
        sys.path.append(scripts_path)
        configs_path = rospack.get_path("distributed_database")
        robot_yaml_path = os.path.join(configs_path, "config", "robotConfigs.yml")
        import database_server_utils as du
        self.__du = du

        # Service configuration
        self.__select_service = "database_server/SelectDB"
        self.__get_hash_service = "database_server/GetDataHashDB"

        self.__select_db = rospy.ServiceProxy(
            self.__select_service, distributed_database.srv.SelectDB
        )

        self.__get_hash_db = rospy.ServiceProxy(
            self.__get_hash_service, distributed_database.srv.GetDataHashDB
        )

        # List of robots to pull
        self.__robot_list = []

        # Publisher creation
        self.publishers = {}
        for t in target.keys():
            robot, topic = re.split(",", t)
            if robot not in self.__robot_list:
                self.__robot_list.append(robot)
            self.publishers[t] = {
                  "pub": rospy.Publisher(f"/{robot}{topic}", target[t], queue_size=10), 
                  "hash_pub": rospy.Publisher(f"/{robot}{topic}_hash", std_msgs.msg.String, queue_size=10)}

    def run(self):
        rate = rospy.Rate(1)
        hashes = []
        while not rospy.is_shutdown():
            for robot in self.__robot_list:
                hashes_to_get = []

                rospy.wait_for_service(self.__select_service)
                try:
                    answ = self.__select_db(robot, -1, -1)
                except rospy.ServiceException as exc:
                    rospy.logerr(f"Service did not process request {exc}")
                    continue
                returned_hashes = answ.hashes

                for hash_ in returned_hashes:
                    if hash_ not in hashes:
                        hashes_to_get.append(hash_)

                for get_hash in hashes_to_get:
                    rospy.wait_for_service(self.__get_hash_service)
                    try:
                        answ = self.__get_hash_db(get_hash)
                    except rospy.ServiceException as exc:
                        print("Service did not process request: " + str(exc))
                        continue
                    hashes.append(get_hash)

                    ans_feat_name, ans_ts, ans_data, _ = self.__du.parse_answer(answ)
                    robot, feat_id, number = re.split(',', ans_feat_name)

                    for t in self.publishers.keys():
                        if t == f"{robot},{feat_id}":
                            self.publishers[t]['pub'].publish(ans_data)
                            self.publishers[t]['hash_pub'].publish(get_hash)
                            rospy.logdebug(f"Publishing {ans_feat_name}")
            rate.sleep()



