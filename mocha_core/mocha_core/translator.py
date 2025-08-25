#!/usr/bin/env python3

import os
import pdb
import sys

import rclpy
import rclpy.time
import yaml
from rclpy.node import Node
import mocha_core.srv
# Get the database utils module
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import mocha_core.database_utils as du
from mocha_core.srv import AddUpdateDB, GetDataHeaderDB, SelectDB
import mocha_core.database_server as ds


class Translator():
    def __init__(
        self, this_robot, this_robot_id, topic_name, topic_id, msg_type, ros_node, callback_group=None
    ):

        self.logger = ros_node.get_logger()
        self.ros_node = ros_node
        self.topic_name = topic_name
        self.topic_id = topic_id
        self.this_robot = this_robot
        self.this_robot_id = this_robot_id

        # Create service client
        self.add_update_db_cli = self.ros_node.create_client(AddUpdateDB,
                                                             "/mocha/add_update_db",
                                                             qos_profile=ds.DatabaseServer.QOS_PROFILE)
        wait_counter = 0
        while not self.add_update_db_cli.wait_for_service(timeout_sec=1.0):
            self.logger.debug("Translator - Waiting for add_update_db")
            wait_counter += 1
            if wait_counter % 5 == 0:
                self.logger.warn("Translator - Waiting for add_update_db for more than 5 seconds")

        # Create subscriber with callback group
        self.subscription = self.ros_node.create_subscription(
            msg_type, self.topic_name, self.translator_cb, 10,
            callback_group=callback_group
        )
        self.logger.info(f"Translator created for {self.topic_name}")

    def translator_cb(self, data):
        msg = data

        serialized_msg = du.serialize_ros_msg(msg)
        # Get current time
        ts = self.ros_node.get_clock().now().to_msg()

        # Create service request
        req = mocha_core.srv.AddUpdateDB.Request()
        req.topic_id = self.topic_id
        req.timestamp = ts
        req.msg_content = serialized_msg

        future = self.add_update_db_cli.call_async(req)
        def response_callback(future):
            if future.done():
                answ = future.result()
                answ_header = answ.new_header
            else:
                self.logger.warning(f"{self.this_robot} - translator " +
                    "Service call failed for {self.topic_name}"
                )
        future.add_done_callback(response_callback)

class TranslatorNode(Node):
    def __init__(self, this_robot=None, robot_configs=None, topic_configs=None):
        super().__init__("translator")
        # Declare parameters
        self.declare_parameter("robot_name", "")
        self.declare_parameter("robot_configs", "")
        self.declare_parameter("topic_configs", "")
        self.logger = self.get_logger()

        # Create reentrant callback group like DatabaseServer
        self.callback_group = ReentrantCallbackGroup()

        self.this_robot = self.get_parameter("robot_name").get_parameter_value().string_value if this_robot is None else this_robot

        if len(self.this_robot) == 0:
            self.logger.error(f"{self.this_robot} - Translator - Empty robot name")
            raise ValueError("Empty robot name")

        # Load and check robot configs
        self.robot_configs_file = self.get_parameter("robot_configs").get_parameter_value().string_value if robot_configs is None else robot_configs
        try:
            with open(self.robot_configs_file, "r") as f:
                self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - Translator - robot_configs file")
            raise e
        if self.this_robot not in self.robot_configs.keys():
            self.logger.error(f"{self.this_robot} - Translator - robot_configs file")
            raise ValueError("Robot not in config file")

        # Load and check topic configs
        self.topic_configs_file = self.get_parameter("topic_configs").get_parameter_value().string_value if topic_configs is None else topic_configs
        try:
            with open(self.topic_configs_file, "r") as f:
                self.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - Translator - topics_configs file")
            raise e
        self_type = self.robot_configs[self.this_robot]["node-type"]
        if self_type not in self.topic_configs.keys():
            self.logger.error(f"{self.this_robot} - Translator - topics_configs file")
            raise ValueError("Node type not in config file")
        this_robot_topics = self.topic_configs[self.robot_configs[self.this_robot]["node-type"]]

        # Get msg_types dict used to publish the topics
        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs,
                                      self)

        # Create translators for each topic
        translators = []
        for topic_id, topic in enumerate(this_robot_topics):
            # Get robot id
            robot_id = du.get_robot_id_from_name(self.robot_configs, self.this_robot)
            obj = self.msg_types[robot_id][topic_id]["obj"]
            translator = Translator(
                self.this_robot, robot_id, topic["msg_topic"], topic_id, obj, self, self.callback_group
            )
            translators.append(translator)

def main(args=None):
    rclpy.init(args=args)
    try:
        translator_node = TranslatorNode()
    except Exception as e:
        print(f"Node initialization failed: {e}")
        rclpy.shutdown()
        return

    # Load mtexecutor
    mtexecutor = MultiThreadedExecutor(num_threads=4)
    mtexecutor.add_node(translator_node)

    try:
        mtexecutor.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    except Exception as e:
        print(f"Exception: {e}")

if __name__ == "__main__":
    main()
