#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
import mocha_core.srv
import yaml
import pdb

class Translator:
    def __init__(self, this_robot, this_robot_id,
                 topic_name, topic_id, msg_type):
        self.__du = du

        self.__counter = 0
        self.__topic_name = topic_name
        self.__topic_id = topic_id
        self.__this_robot = this_robot
        self.__this_robot_id = this_robot_id
        self.__service_name = "integrate_database/AddUpdateDB"
        self.__add_update_db = rospy.ServiceProxy(
                self.__service_name, mocha_core.srv.AddUpdateDB
        )

        rospy.Subscriber(
            self.__topic_name, msg_type, self.translator_cb
        )

    def translator_cb(self, data):
        msg = data

        rospy.wait_for_service(self.__service_name)
        serialized_msg = self.__du.serialize_ros_msg(msg)
        try:
            ts = rospy.get_rostime()
            answ = self.__add_update_db(self.__topic_id,
                                        ts,
                                        serialized_msg)
            answ_header = answ.new_header
            rospy.logdebug(f"{self.__this_robot} - Header insert " +
                           f"- {self.__topic_name} - {answ_header}")
            self.__counter += 1
        except rospy.ServiceException as exc:
            rospy.logerr(f"Service did not process request: {exc}")


if __name__ == "__main__":
    rospy.init_node("topic_translator", anonymous=False)

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
    topic_configs_path = rospy.get_param("~topic_configs",
                                         topic_configs_default)
    with open(topic_configs_path, 'r') as f:
        topic_configs = yaml.load(f, Loader=yaml.FullLoader)
    this_robot_topics = topic_configs[robot_configs[this_robot]["node-type"]]

    # Get msg_types dict used to publish the topics
    msg_types = du.msg_types(robot_configs, topic_configs)

    for topic_id, topic in enumerate(this_robot_topics):
        # Get robot id
        robot_id = du.get_robot_id_from_name(robot_configs, this_robot)
        obj = msg_types[robot_id][topic_id]["obj"]
        Translator(this_robot,
                   robot_id,
                   topic["msg_topic"],
                   topic_id,
                   obj)
    rospy.spin()
