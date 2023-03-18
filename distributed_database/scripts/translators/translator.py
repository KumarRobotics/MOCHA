#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
import distributed_database.srv
import yaml
import pdb

PRIORITY_LUT = {"NO_PRIORITY": 0,
                "LOW_PRIORITY": 1,
                "MEDIUM_PRIORITY": 2,
                "HIGH_PRIORITY": 3}

class Translator:
    def __init__(self, this_robot, topic_name, prio, msg_type):
        self.__du = du

        self.__counter = 0
        self.__topic_name = topic_name
        self.__this_robot = this_robot
        self.__service_name = "integrate_database/AddUpdateDB"
        self.__add_update_db = rospy.ServiceProxy(
                self.__service_name, distributed_database.srv.AddUpdateDB
        )
        self.__prio = prio

        rospy.Subscriber(
            self.__topic_name, msg_type, self.translator_cb
        )

    def translator_cb(self, data):
        msg = data

        feature_name = f"{self.__this_robot},{self.__topic_name},{self.__counter}"

        rospy.wait_for_service(self.__service_name)
        serialized_msg = self.__du.serialize_ros_msg(msg)
        try:
            answ = self.__add_update_db(feature_name,
                                        msg._md5sum,
                                        self.__prio,
                                        serialized_msg)
            answ_hash = answ.new_hash
            rospy.logdebug(f"{self.__this_robot} - Hash insert " +
                           f"- {self.__topic_name} - {answ_hash}")
            self.__counter += 1
        except rospy.ServiceException as exc:
            rospy.logerr(f"Service did not process request: {exc}")


if __name__ == "__main__":
    rospy.init_node("topic_translator", anonymous=False)

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
    topic_configs_path = rospy.get_param("~topic_configs",
                                         topic_configs_default)
    with open(topic_configs_path, 'r') as f:
        topic_configs = yaml.load(f, Loader=yaml.FullLoader)
    this_robot_topics = topic_configs[robot_configs[this_robot]["node-type"]]

    # Get msg_types dict used to publish the topics
    msg_types = du.msg_types(topic_configs)

    # Flip the dict so that we have name: obj
    msg_objects = {}
    for msg in msg_types:
        msg_objects[msg_types[msg]["name"]] = msg_types[msg]["obj"]

    for topic in this_robot_topics:
        # Translate priority into a number
        if topic["msg_priority"] not in PRIORITY_LUT.keys():
            rospy.logerr(f"Error: msg_prio {topic['msg_priority']}" +
                         f" not in {PRIORITY_LUT.keys()}")
            rospy.signal_shutdown("Error: msg_priority not valid")
            rospy.spin()
        prio = PRIORITY_LUT[topic["msg_priority"]]
        Translator(this_robot,
                   topic["msg_topic"],
                   prio,
                   msg_objects[topic["msg_type"]])
    rospy.spin()
