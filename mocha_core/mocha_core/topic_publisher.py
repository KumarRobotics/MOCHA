#!/usr/bin/env python3
import os
import pdb
import re
import sys

import rclpy
import rclpy.clock
import rclpy.time
from rclpy.node import Node
import yaml
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import mocha_core.database_utils as du
import mocha_core.msg
import mocha_core.srv
from mocha_core.srv import AddUpdateDB, GetDataHeaderDB, SelectDB
from mocha_core.msg import DatabaseCB
import mocha_core.database_server as ds
import mocha_core.hash_comm as hc
import queue
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_ALL,
)


class TopicPublisher:
    def __init__(self, robot_id, robot_name, topic_id, topic_name, msg_types, obj, ros_node):
        assert robot_id is not None and isinstance(robot_id, int)
        assert topic_id is not None and isinstance(topic_id, int)
        assert ros_node is not None
        assert msg_types is not None

        self.robot_id = robot_id
        self.robot_name = robot_name
        self.topic_id = topic_id
        self.topic_name = topic_name
        self.msg_types = msg_types
        self.ros_node = ros_node
        self.logger = self.ros_node.get_logger()
        self.get_data_header_db_cli = ros_node.get_data_header_db_cli
        # self.header_pub = ros_node.header_pub

        self.publisher = self.ros_node.create_publisher(obj,
                                                        f"/{robot_name}{topic_name}",
                                                        qos_profile=QOS_PROFILE)
        self.is_shutdown = threading.Event()
        self.is_shutdown.set()

        self.msg_queue = queue.Queue()

        self.th = threading.Thread(target=self.run, args=())


    def run(self):
        self.logger.info(f"{self.robot_name}{self.topic_name} - Topic Publisher - Started")
        self.is_shutdown.clear()
        while not self.is_shutdown.is_set():
            # Try to acquire the mutex and loop back if we failed
            try:
                msg_header = self.msg_queue.get(timeout=1)
            except queue.Empty:
                continue
            self.logger.debug(f"{self.robot_name}{self.topic_name} - Topic Publisher - Loop")
            req = mocha_core.srv.GetDataHeaderDB.Request()
            req.msg_header = msg_header
            future = self.get_data_header_db_cli.call_async(req)

            def handle_response(future):
                try:
                    if future.done():
                        answ = future.result()
                        ans_robot_id, ans_topic_id, ans_ts, ans_data = du.parse_answer(
                            answ, self.msg_types
                        )
                        self.publisher.publish(ans_data)
                        # self.msg_queue.task_done()
                    else:
                        self.logger.warning(f"{self.robot_name} - Topic Publisher - get_data_header_db service call timeout")
                except Exception as e:
                    self.logger.error(f"{self.robot_name} - Topic Publisher - Error handling service response: {e}")

            future.add_done_callback(handle_response)

    def get_message(self, msg_header):
        self.logger.debug(f"{self.robot_name} - Topic Publisher - Callback")
        self.msg_queue.put(msg_header)

    def shutdown(self):
        # self.msg_queue.join()
        self.is_shutdown.set()

class TopicPublisherNode(Node):
    def __init__(self, this_robot=None, robot_configs=None, topic_configs=None):
        super().__init__("topic_publisher")
        # Declare parameters
        self.declare_parameter("robot_name", "")
        self.declare_parameter("robot_configs", "")
        self.declare_parameter("topic_configs", "")
        self.logger = self.get_logger()

        # Create reentrant callback group like DatabaseServer
        self.callback_group = ReentrantCallbackGroup()

        self.this_robot = self.get_parameter("robot_name").get_parameter_value().string_value if this_robot is None else this_robot

        if len(self.this_robot) == 0:
            self.logger.error(f"{self.this_robot} - Topic Publisher - Empty robot name")
            raise ValueError("Empty robot name")

        # Load and check robot configs
        self.robot_configs_file = self.get_parameter("robot_configs").get_parameter_value().string_value if robot_configs is None else robot_configs
        try:
            with open(self.robot_configs_file, "r") as f:
                self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - Topic Publisher - robot_configs file")
            raise e
        if self.this_robot not in self.robot_configs.keys():
            self.logger.error(f"{self.this_robot} - Topic Publisher - robot_configs file")
            raise ValueError("Robot not in config file")

        # Load and check topic configs
        self.topic_configs_file = self.get_parameter("topic_configs").get_parameter_value().string_value if topic_configs is None else topic_configs
        try:
            with open(self.topic_configs_file, "r") as f:
                self.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - Topic Publisher - topics_configs file")
            raise e
        self_type = self.robot_configs[self.this_robot]["node-type"]
        if self_type not in self.topic_configs.keys():
            self.logger.error(f"{self.this_robot} - Topic Publisher - topics_configs file")
            raise ValueError("Node type not in config file")
        this_robot_topics = self.topic_configs[self.robot_configs[self.this_robot]["node-type"]]

        # Get msg_types dict used to publish the topics
        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs,
                                      self)

        # Create the service clients
        self.get_data_header_db_cli = self.create_client(GetDataHeaderDB,
                                                         "/mocha/get_data_header_db",
                                                         qos_profile=ds.DatabaseServer.QOS_PROFILE)
        wait_counter = 0
        while not self.get_data_header_db_cli.wait_for_service(timeout_sec=1.0):
            self.logger.debug("Topic Publisher - Waiting for get_data_header_db")
            wait_counter += 1
            if wait_counter % 5 == 0:
                self.logger.warn("Topic Publisher - Waiting for get_data_header_db for more than 5 seconds")

        # Create a service to publish the received headers
        # self.header_pub = self.create_publisher(
        #     mocha_core.msg.HeaderPub, "topic_publisher/headers", 10
        # )

        self.list_of_threads = {}
        # Iterate for each robot in robot_configs, and generate the threads
        for robot in self.robot_configs.keys():
            robot_id = du.get_robot_id_from_name(self.robot_configs, robot)
            if robot_id not in self.list_of_threads:
                self.list_of_threads[robot_id] = {}
            robot_type = self.robot_configs[robot]["node-type"]
            topic_list = self.topic_configs[robot_type]
            for topic in topic_list:
                msg_topic = topic["msg_topic"]
                topic_id = du.get_topic_id_from_name(self.robot_configs,
                                                     self.topic_configs,
                                                     robot,
                                                     msg_topic,
                                                     self)
                if topic_id not in self.list_of_threads[robot_id]:
                    self.list_of_threads[robot_id][topic_id] = {}

                obj = self.msg_types[robot_id][topic_id]["obj"]
                tp = TopicPublisher(robot_id, robot, topic_id, msg_topic,
                                    self.msg_types, obj, self)
                self.list_of_threads[robot_id][topic_id] = tp
                tp.th.start()
        sub = self.create_subscription(DatabaseCB, "/mocha/database_cb",
                                       self.msgheader_callback, 10,
                                       callback_group=self.callback_group)

    def msgheader_callback(self, data):
        robot_id = data.robot_id
        topic_id = data.topic_id
        msg_header = data.msg_header

        # Signal the right thread
        self.list_of_threads[robot_id][topic_id].get_message(msg_header)


    def shutdown(self):
        for topic_ids in self.list_of_threads.values():
            for tp in topic_ids.values():
                tp.shutdown()
        for topic_ids in self.list_of_threads.values():
            for tp in topic_ids.values():
                tp.th.join()


def main(args=None):
    # Initialize ROS2
    rclpy.init()
    try:
        topic_publisher_node = TopicPublisherNode()
    except Exception as e:
        print(f"Node initialization failed: {e}")
        rclpy.shutdown()
        return

    # Load mtexecutor
    mtexecutor = MultiThreadedExecutor(num_threads=4)
    mtexecutor.add_node(topic_publisher_node)

    try:
        mtexecutor.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        topic_publisher_node.shutdown()
    except Exception as e:
        print(f"Exception: {e}")
        topic_publisher_node.shutdown()

if __name__ == "__main__":
    main()
