#!/usr/bin/env python3
import os
import pdb
import sys

import rospkg
import rospy
import yaml
import re
import mocha_core.msg

import mocha_core.srv


class TopicPublisher():
    def __init__(self, this_robot, target, msg_history="WHOLE_HISTORY"):

        self.this_robot = this_robot


        # Service configuration
        self.__select_service = "integrate_database/SelectDB"
        self.__get_header_service = "integrate_database/GetDataHeaderDB"

        try:
            rospy.wait_for_service(self.__select_service)
            rospy.wait_for_service(self.__get_header_service)
        except rospy.ROSInterruptException as exc:
            rospy.logdebug("Service did not process request: " +
                           str(exc))
            rospy.signal_shutdown("Service did not process request")
            rospy.spin()
        self.__select_db = rospy.ServiceProxy(
            self.__select_service, mocha_core.srv.SelectDB
        )

        self.__get_header_db = rospy.ServiceProxy(
            self.__get_header_service, mocha_core.srv.GetDataHeaderDB
        )

        # List of robots to pull
        self.__robot_list = []

        # Publisher creation
        self.publishers = {}
        self.header_pub = rospy.Publisher("ddb/topic_publisher/headers",
                                          mocha_core.msg.Header_pub,
                                          queue_size=10)
        for t in target:
            robot, robot_id, topic, topic_id, obj = t
            if robot not in self.__robot_list:
                self.__robot_list.append(robot_id)
            self.publishers[(robot_id, topic_id)] = {"pub": rospy.Publisher(f"/{robot}{topic}",
                                                         obj,
                                                         queue_size=10),
                                                         "ts": rospy.Time(1, 0)}


    def run(self):
        rospy.loginfo(f"{self.this_robot} - Topic Publisher - Started")
        rate = rospy.Rate(10)
        headers = set()
        while not rospy.is_shutdown():
            for robot_id in self.__robot_list:
                headers_to_get = []

                try:
                    answ = self.__select_db(robot_id, None, None)
                except rospy.ServiceException as exc:
                    rospy.logdebug(f"Service did not process request {exc}")
                    continue
                returned_headers = du.deserialize_headers(answ.headers)
                if len(returned_headers) == 0:
                    rate.sleep()
                    continue

                for header_ in returned_headers:
                    if header_ not in headers:
                        headers_to_get.append(header_)

                for get_header in headers_to_get:
                    rospy.logdebug(f"Getting headers {get_header}")
                    try:
                        answ = self.__get_header_db(get_header)
                    except rospy.ServiceException as exc:
                        rospy.logerr("Service did not process request: " +
                                     str(exc))
                        continue
                    headers.add(get_header)

                    ans_robot_id, ans_topic_id, ans_ts, ans_data = du.parse_answer(answ,
                                                                                   msg_types)
                    for t in self.publishers.keys():
                        if t == (ans_robot_id, ans_topic_id):
                            assert isinstance(ans_data,
                                              self.publishers[t]['pub'].data_class)
                            # FIXME: remove this line once we have proper time
                            # filtering implemented
                            if ans_ts > self.publishers[t]["ts"]:
                                self.publishers[t]["ts"] = ans_ts
                                self.publishers[t]["pub"].publish(ans_data)
                                self.header_pub.publish(get_header)
                                rospy.logdebug(f"Publishing robot_id: {ans_robot_id}" +
                                               f" - topic: {ans_topic_id}")
                            else:
                                rospy.logdebug(f"Skipping robot_id: {ans_robot_id}" +
                                               f" - topic: {ans_topic_id} as there is an old ts")
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mocha_core_publisher", anonymous=False)

    # Get the mocha_core path
    rospack = rospkg.RosPack()
    ddb_path = rospack.get_path('mocha_core')
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
    msg_types = du.msg_types(robot_configs, topic_configs)

    # Flip the dict so that we have name: obj
    msg_objects = {}
    # for msg in msg_types:
    #     msg_objects[msg_types[msg]["name"]] = msg_types[msg]["obj"]

    list_of_topics = set()

    # Iterate for each robot in robot_configs, and generate the topics
    for robot in robot_configs.keys():
        robot_id = du.get_robot_id_from_name(robot_configs, robot)
        robot_type = robot_configs[robot]["node-type"]
        topic_list = topic_configs[robot_type]
        for topic_id, topic in enumerate(topic_list):
            msg_topic = topic["msg_topic"]
            obj = msg_types[robot_id][topic_id]["obj"]
            robot_tuple = (robot, robot_id, topic["msg_topic"], topic_id, obj)
            list_of_topics.add(robot_tuple)

    pub = TopicPublisher(this_robot, list_of_topics)
    pub.run()
