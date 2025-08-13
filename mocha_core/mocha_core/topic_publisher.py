#!/usr/bin/env python3
import os
import pdb
import sys

import rclpy
import rclpy.time
import rclpy.clock
import ament_index_python
import yaml
import re
import mocha_core.msg

import mocha_core.srv


class TopicPublisher():
    def __init__(self, this_robot, target, msg_history="WHOLE_HISTORY", node=None):

        self.this_robot = this_robot

        # Store or create the ROS2 node
        if node is None:
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()
            self.__node = rclpy.create_node(f'topic_publisher_{this_robot}')
            self.__node_owned = True
        else:
            self.__node = node
            self.__node_owned = False
        
        self.__logger = self.__node.get_logger()

        # Service configuration
        self.__select_service = "select_db"
        self.__get_header_service = "get_data_header_db"

        # Create service clients
        self.__select_db = self.__node.create_client(
            mocha_core.srv.SelectDB, self.__select_service
        )
        self.__get_header_db = self.__node.create_client(
            mocha_core.srv.GetDataHeaderDB, self.__get_header_service
        )

        # Wait for services to be available
        if not self.__select_db.wait_for_service(timeout_sec=10.0):
            self.__logger.error(f"Service {self.__select_service} not available")
            
        if not self.__get_header_db.wait_for_service(timeout_sec=10.0):
            self.__logger.error(f"Service {self.__get_header_service} not available")

        # List of robots to pull
        self.__robot_list = []

        # Publisher creation
        self.publishers = {}
        self.header_pub = self.__node.create_publisher(
            mocha_core.msg.HeaderPub,
            "ddb/topic_publisher/headers",
            10
        )
        for t in target:
            robot, robot_id, topic, topic_id, obj = t
            if robot not in self.__robot_list:
                self.__robot_list.append(robot_id)
            self.publishers[(robot_id, topic_id)] = {
                "pub": self.__node.create_publisher(obj, f"/{robot}{topic}", 10),
                "ts": rclpy.time.Time(seconds=1, nanoseconds=0)
            }


    def run(self):
        self.__logger.info(f"{self.this_robot} - Topic Publisher - Started")
        rate = self.__node.create_rate(10)
        headers = set()
        while rclpy.ok():
            for robot_id in self.__robot_list:
                headers_to_get = []

                # Create service request
                req = mocha_core.srv.SelectDB.Request()
                req.robot_id = robot_id
                req.topic_id = 0  # None equivalent for uint8
                req.ts_limit = self.__node.get_clock().now().to_msg()
                
                try:
                    future = self.__select_db.call_async(req)
                    rclpy.spin_until_future_complete(self.__node, future, timeout_sec=1.0)
                    if future.done():
                        answ = future.result()
                    else:
                        self.__logger.debug("Service call timeout")
                        continue
                except Exception as exc:
                    self.__logger.debug(f"Service did not process request {exc}")
                    continue
                returned_headers = du.deserialize_headers(answ.headers)
                if len(returned_headers) == 0:
                    rate.sleep()
                    continue

                for header_ in returned_headers:
                    if header_ not in headers:
                        headers_to_get.append(header_)

                for get_header in headers_to_get:
                    self.__logger.debug(f"Getting headers {get_header}")
                    
                    # Create service request for getting header data
                    req = mocha_core.srv.GetDataHeaderDB.Request()
                    req.msg_header = get_header
                    
                    try:
                        future = self.__get_header_db.call_async(req)
                        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=1.0)
                        if future.done():
                            answ = future.result()
                        else:
                            self.__logger.debug("Get header service call timeout")
                            continue
                    except Exception as exc:
                        self.__logger.error("Service did not process request: " +
                                           str(exc))
                        continue
                    headers.add(get_header)

                    ans_robot_id, ans_topic_id, ans_ts, ans_data = du.parse_answer(answ,
                                                                                   msg_types)
                    for t in self.publishers.keys():
                        if t == (ans_robot_id, ans_topic_id):
                            # Convert ROS1 timestamp to ROS2 Time for comparison
                            ans_ts_ros2 = rclpy.time.Time.from_msg(ans_ts) if hasattr(ans_ts, 'sec') else ans_ts
                            current_ts = self.publishers[t]["ts"]
                            
                            # FIXME: remove this line once we have proper time
                            # filtering implemented
                            if ans_ts_ros2 > current_ts:
                                self.publishers[t]["ts"] = ans_ts_ros2
                                self.publishers[t]["pub"].publish(ans_data)
                                self.header_pub.publish(get_header)
                                self.__logger.debug(f"Publishing robot_id: {ans_robot_id}" +
                                                   f" - topic: {ans_topic_id}")
                            else:
                                self.__logger.debug(f"Skipping robot_id: {ans_robot_id}" +
                                                   f" - topic: {ans_topic_id} as there is an old ts")
                rate.sleep()

    def shutdown(self):
        """Cleanup method for shutting down the topic publisher"""
        if self.__node_owned and self.__node is not None:
            self.__node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def create_topic_publisher_node(robot_name, robot_configs_path=None, topic_configs_path=None, node=None):
    """
    Create and setup topic publisher for a given robot.
    Returns the node for cleanup.
    """
    # Use provided node or create new one
    if node is None:
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node(f"topic_publisher_{robot_name}")
        node_owned = True
    else:
        node_owned = False
        
    logger = node.get_logger()

    # Get the mocha_core path using ament
    try:
        from ament_index_python.packages import get_package_share_directory
        ddb_path = get_package_share_directory('mocha_core')
    except:
        # Fallback for development
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ddb_path = os.path.join(current_dir, "..")
    
    # Add path for database_utils
    sys.path.append(os.path.join(ddb_path, "."))
    import database_utils as du

    # Get the robot_config path and generate the robot_configs object
    if robot_configs_path is None:
        robot_configs_path = os.path.join(ddb_path, "config", "testConfigs", "robot_configs.yaml")
        
    with open(robot_configs_path, 'r') as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)
    if robot_name not in robot_configs.keys():
        logger.error(f"Robot {robot_name} not in robot_configs")
        if node_owned:
            rclpy.shutdown()
        raise ValueError(f"Robot {robot_name} not found in config")

    # Get the topic_configs path and generate the topic_configs object
    if topic_configs_path is None:
        topic_configs_path = os.path.join(ddb_path, "config", "testConfigs", "topic_configs.yaml")
        
    with open(topic_configs_path, 'r') as f:
        topic_configs = yaml.load(f, Loader=yaml.FullLoader)

    # Get msg_types dict used to publish the topics
    msg_types = du.msg_types(robot_configs, topic_configs)

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

    # Create TopicPublisher instance
    pub = TopicPublisher(robot_name, list_of_topics, node=node)
    
    return node, pub


if __name__ == "__main__":
    # Initialize ROS2
    rclpy.init()
    
    # Create main node
    node = rclpy.create_node("mocha_core_publisher")
    logger = node.get_logger()

    # Declare parameters with defaults
    node.declare_parameter("robot_name", "")
    node.declare_parameter("robot_configs", "")
    node.declare_parameter("topic_configs", "")

    # Get robot from the parameters
    this_robot = node.get_parameter("robot_name").get_parameter_value().string_value
    if not this_robot:
        logger.error("robot_name parameter is required")
        rclpy.shutdown()
        sys.exit(1)

    # Get config file paths from parameters
    robot_configs_path = node.get_parameter("robot_configs").get_parameter_value().string_value
    topic_configs_path = node.get_parameter("topic_configs").get_parameter_value().string_value
    
    # Use empty strings as None for the function
    robot_configs_path = robot_configs_path if robot_configs_path else None
    topic_configs_path = topic_configs_path if topic_configs_path else None

    try:
        # Create topic publisher node and publisher instance
        _, pub = create_topic_publisher_node(this_robot, robot_configs_path, topic_configs_path, node)
        
        # Run the publisher
        pub.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        pub.shutdown()
        node.destroy_node()
        rclpy.shutdown()
