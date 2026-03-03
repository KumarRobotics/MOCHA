#!/usr/bin/env python3

import os
import threading
import rclpy
import rclpy.logging
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup
import mocha_core.srv
import mocha_core.database as database
import pdb
import mocha_core.database_utils as du
import mocha_core.msg

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class DatabaseServer:
    """ Starts a clean database and offers an API-like service
    to interact with the database.

    Please see the list of services in the srv folder
    """
    QOS_PROFILE = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_ALL,
    )

    def __init__(self, robot_configs, topic_configs, robot_name, ros_node):
        # Check input topics
        assert isinstance(robot_configs, dict)
        assert isinstance(topic_configs, dict)
        assert isinstance(robot_name, str)
        assert ros_node is not None

        self.robot_configs = robot_configs
        self.topic_configs = topic_configs

        # Store or create the ROS2 node
        self.ros_node = ros_node
        self.logger = self.ros_node.get_logger()
        self.ros_node_name = self.ros_node.get_fully_qualified_name()

        self.callback_group = ReentrantCallbackGroup()

        # Get robot name from parameter
        self.robot_name = robot_name

        if self.robot_name not in self.robot_configs.keys():
            self.logger.error(f"{self.robot_name} does not exist in robot_configs")
            raise ValueError("robot_name does not exist in robot_configs")

        self.topic_list = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]

        # Publish a message every time that the database is modified
        pub_database = self.ros_node.create_publisher(
            mocha_core.msg.DatabaseCB,
            f"{self.ros_node_name}/database_cb",
            10
        )
        def database_cb(robot_id, topic_id, ts, header):
            database_cb_msg = mocha_core.msg.DatabaseCB()
            database_cb_msg.header.stamp = ts.to_msg()
            database_cb_msg.header.frame_id = self.robot_name
            database_cb_msg.robot_id = robot_id
            database_cb_msg.topic_id = topic_id
            database_cb_msg.msg_header = header
            pub_database.publish(database_cb_msg)

        # Create the empty database with lock
        self.dbl = database.DBwLock(ros_node=self.ros_node, insertion_cb=database_cb)

        # Get current robot number
        self.robot_number = du.get_robot_id_from_name(self.robot_configs,
                                                      robot_name=self.robot_name)

        # create services for all the possible calls to the DB
        self.service_list = []
        self.logger.debug("database_server: Starting database server")
        s = self.ros_node.create_service(mocha_core.srv.AddUpdateDB,
                                    f"{self.ros_node_name}/add_update_db",
                                    self.add_update_db_service_cb,
                                    callback_group=self.callback_group,
                                    qos_profile=self.QOS_PROFILE)
        self.service_list.append(s)
        s = self.ros_node.create_service(mocha_core.srv.GetDataHeaderDB,
                                    f"{self.ros_node_name}/get_data_header_db",
                                    self.get_data_hash_db_service_cb,
                                    callback_group=self.callback_group,
                                    qos_profile=self.QOS_PROFILE)
        self.service_list.append(s)
        s = self.ros_node.create_service(mocha_core.srv.SelectDB,
                                    f"{self.ros_node_name}/select_db",
                                    self.select_db_service_cb,
                                    callback_group=self.callback_group,
                                    qos_profile=self.QOS_PROFILE)
        self.service_list.append(s)
        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs,
                                      self.ros_node)

    def add_update_db_service_cb(self, req, response):
        if not isinstance(req.topic_id, int) or req.topic_id is None:
            self.logger.error("topic_id empty")
            return response
        if len(req.msg_content) == 0:
            self.logger.error("msg_content empty")
            return response
        if req.topic_id >= len(self.topic_list):  # Changed > to >= for proper bounds check
            self.logger.error(f"topic_id {req.topic_id} not in topic_list (length: {len(self.topic_list)})")
            return response

        topic = self.topic_list[req.topic_id]
        priority = du.get_priority_number(topic["msg_priority"])
        # req.timestamp is already a builtin_interfaces.msg.Time
        # Convert to rclpy.time.Time for internal processing
        ts_msg = req.timestamp
        ts = rclpy.time.Time.from_msg(ts_msg)

        # Convert array to bytes if needed (ROS2 service messages use array)
        msg_data = req.msg_content
        msg_data = bytes(msg_data)

        dbm = database.DBMessage(self.robot_number,
                                 req.topic_id,
                                 dtype=self.msg_types[self.robot_number][req.topic_id]["dtype"],
                                 priority=priority,
                                 ts=ts,
                                 data=msg_data)

        header = self.dbl.add_modify_data(dbm)
        response.new_header = header
        return response

    def get_data_hash_db_service_cb(self, req, response):
        if req.msg_header is None or len(req.msg_header) == 0:
            self.logger.error("msg_header empty")
            return response

        # Convert array to bytes if needed (ROS2 service messages use array)
        header_data = req.msg_header
        header_data = bytes(header_data)

        dbm = self.dbl.find_header(header_data)

        response.robot_id = dbm.robot_id
        response.topic_id = dbm.topic_id
        # Convert rclpy.time.Time to builtin_interfaces.msg.Time
        response.timestamp = dbm.ts.to_msg()
        response.msg_content = dbm.data
        return response

    def select_db_service_cb(self, req, response):
        # TODO Implement filtering

        list_headers = self.dbl.get_header_list(req.robot_id)

        response.headers = du.serialize_headers(list_headers)
        return response
