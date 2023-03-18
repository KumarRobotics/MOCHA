#!/usr/bin/env python3
import os
import pdb
import sys

import rospkg
import rospy
import yaml
import re
import std_msgs.msg

import distributed_database.srv


#    def cb(self, data):
#        # rospy.logwarn(f"cb: {self._topic_name}")
#        if self._most_recent is None or data.header.stamp > self._most_recent:
#            self._most_recent = data.header.stamp
#            self._pub.publish(data)
#            # rospy.logwarn(f"publishing: {self._topic_name} - {self._most_recent}")

class TopicPublisher(msg_history="WHOLE_HISTORY"):
    def __init__(self, target):
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
                  "pub": rospy.Publisher(f"/{robot}{topic}",
                                         target[t], queue_size=10), 
                  "hash_pub": rospy.Publisher(f"/{robot}{topic}_hash",
                                              std_msgs.msg.String, queue_size=10)}

    def run(self):
        rospy.loginfo("Starting TopicPublisher")
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
                        rospy.logerr("Service did not process request: " +
                                     str(exc))
                        continue
                    hashes.append(get_hash)

                    ans_feat_name, ans_ts, ans_data, _ = self.__du.parse_answer(answ)
                    robot, feat_id, number = re.split(',', ans_feat_name)

                    for t in self.publishers.keys():
                        if t == f"{robot}, {feat_id}":
                            assert isinstance(ans_data,
                                              self.publishers[t]['pub'].data_class)
                            self.publishers[t]['pub'].publish(ans_data)
                            self.publishers[t]['hash_pub'].publish(get_hash)
                            rospy.logdebug(f"Publishing {ans_feat_name}")
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("distributed_database_publisher", anonymous=True)

    # Get the distributed_database path
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(ddb_path, "scripts/core")
    sys.path.append(scripts_path)
    import database_utils as du

    # Get robot from the parameters
    this_robot = rospy.get_param("~robot_name")

    # Get the robot_config path and generate the robot_configs object
    robot_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/robot_configs.yaml")
    robot_configs_path = rospy.get_param("~robot_configs", robot_configs_default)
    with open(robot_configs_path, 'r') as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)
    if this_robot not in robot_configs.keys():
        rospy.logerr(f"Robot {this_robot} not in robot_configs")
        rospy.signal_shutdown("Robot not in robot_configs")
        rospy.spin()


    # Get the topic_configs path and generate the topic_configs object
    topic_configs_default = os.path.join(ddb_path,
                                         "config/testConfigs/topic_configs.yaml")
    topic_configs_path = rospy.get_param("~topic_configs", topic_configs_default)
    with open(topic_configs_path, 'r') as f:
        topic_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Get msg_types dict used to publish the topics
    msg_types = du.msg_types(topic_configs)

    # Flip the dict so that we have name: obj
    msg_objects = {}
    for msg in msg_types:
        msg_objects[msg_types[msg]["name"]] = msg_types[msg]["obj"]

    list_of_topics = {}

    # Iterate for each robot in robot_configs, and generate the topics
    for robot in robot_configs.keys():
        robot_type = robot_configs[robot]["node-type"]
        topic_list = topic_configs[robot_type]
        for topic in topic_list:
            msg_topic = topic["msg_topic"]
            # join all strings with a /, stripping the leading / from the topic 
            topic_to_publish = ",".join([robot, msg_topic])
            msg_type = msg_objects[topic["msg_type"]]
            # msg_obj = msg_objects[msg_type]
            list_of_topics[topic_to_publish] = msg_type

    pub = TopicPublisher(list_of_topics)
    pub.run()
