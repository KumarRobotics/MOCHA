import os
import pdb
import sys

import rospkg
import rospy
import yaml

import distributed_database.srv
import nav_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

class TopicPublisher():
    def __init__(self, topic_dict):
        # Imports
        rospack = rospkg.RosPack()
        comms_path = rospack.get_path("distributed_database")
        scripts_path = os.path.join(comms_path, "scripts")
        sys.path.append(scripts_path)
        configs_path = rospack.get_path("network_configs")
        robot_yaml_path = os.path.join(configs_path, "config", "robotConfigs.yml")
        import database_server_utils as du

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
        with open(robot_yaml_path, "r") as f:
            robot_cfg = yaml.load(f)
        self.__robot_list = list(robot_cfg.keys())

        # Publisher creation
        self.__publishers = {}
        for topic in topic_dict.keys():
            self.__publishers[topic] = {"pub:": rospy.Publisher(
                topic, topic_dict[topic], queue_size=10
            ), "hash_pub":
        rospy.Publisher(
            topic + "_hash", std_msgs.msg.String, queue_size=10
        )

    def run(self):
        rate = rospy.Rate(.2)
        while not rospy.is_shutdown():
            for robot in self.__robot_list:
                hashes_to_get = []

                rospy.wait_for_service(self.__select_service)
                try:
                    answ = self.__select_db(robot, -1, -1)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
                returned_hashes = answ.hashes

                for hash_ in returned_hashes:
                    if hash_ not in hashes:
                        hashes_to_get.append(hash_)

                for get_hash in hashes_to_get:
                    rospy.wait_for_service(self.__get_hash_service)
                    try:
                        answ = self.get_hash_db(get_hash)
                        hashes.append(get_hash)
                    except rospy.ServiceException as exc:
                        print("Service did not process request: " + str(exc))
                        continue

                    ans_feat_name, ans_ts, ans_data, _ = du.parse_answer(answ)

                    for topic in topic_dict.keys():
                        if topic in ans_feat_name:
                            self.__publishers[topic]["pub"].publish(ans_data)
                            self.__publishers[topic]["hash_pub"].publish(get_hash)
                            rospy.logdebug(f"Publishing {ans_feat_name}")
        rate.sleep()



