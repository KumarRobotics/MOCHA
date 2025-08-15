#!/usr/bin/env python3

import os
import sys
import rclpy
import rclpy.time
import ament_index_python
import mocha_core.srv
import yaml
import pdb

class Translator:
    def __init__(self, this_robot, this_robot_id,
                 topic_name, topic_id, msg_type, node=None):
        # Get the database utils module
        import mocha_core.database_utils as du
        self.__du = du

        # Store or create the ROS2 node
        if node is None:
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()
            self.__node = rclpy.create_node(f'translator_{this_robot}_{topic_id}')
            self.__node_owned = True
        else:
            self.__node = node
            self.__node_owned = False
        
        self.__logger = self.__node.get_logger()

        self.__counter = 0
        self.__topic_name = topic_name
        self.__topic_id = topic_id
        self.__this_robot = this_robot
        self.__this_robot_id = this_robot_id
        self.__service_name = "/mocha_core/add_update_db"
        
        # Create service client
        self.__add_update_db = self.__node.create_client(
            mocha_core.srv.AddUpdateDB, self.__service_name
        )

        # Create subscriber
        self.__subscription = self.__node.create_subscription(
            msg_type, self.__topic_name, self.translator_cb, 10
        )

    def translator_cb(self, data):
        msg = data

        # Wait for service to be available
        if not self.__add_update_db.wait_for_service(timeout_sec=5.0):
            self.__logger.error(f"Service {self.__service_name} not available")
            return
        
        serialized_msg = self.__du.serialize_ros_msg(msg)
        try:
            # Get current time
            ts = self.__node.get_clock().now().to_msg()
            
            # Create service request
            req = mocha_core.srv.AddUpdateDB.Request()
            req.topic_id = self.__topic_id
            req.timestamp = ts
            req.msg_content = serialized_msg
            
            # Call service asynchronously
            future = self.__add_update_db.call_async(req)
            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=2.0)
            
            if future.done():
                answ = future.result()
                answ_header = answ.new_header
                self.__logger.debug(f"{self.__this_robot} - Header insert " +
                               f"- {self.__topic_name} - {answ_header}")
                self.__counter += 1
            else:
                self.__logger.warning(f"Service call timed out for {self.__topic_name}")
                
        except Exception as exc:
            self.__logger.error(f"Service did not process request: {exc}")
    
    def shutdown(self):
        """Cleanup method for shutting down the translator"""
        if self.__node_owned and self.__node is not None:
            self.__node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def create_translator_node(robot_name, robot_configs_path=None, topic_configs_path=None, node=None):
    """
    Create and run translator node for a given robot.
    Returns the node for cleanup.
    """
    # Use provided node or create new one
    if node is None:
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node(f"translator_{robot_name}")
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
    
    # Import database_utils from installed package
    import mocha_core.database_utils as du

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
    this_robot_topics = topic_configs[robot_configs[robot_name]["node-type"]]

    # Get msg_types dict used to publish the topics
    msg_types = du.msg_types(robot_configs, topic_configs)

    # Create translators for each topic
    translators = []
    for topic_id, topic in enumerate(this_robot_topics):
        # Get robot id
        robot_id = du.get_robot_id_from_name(robot_configs, robot_name)
        obj = msg_types[robot_id][topic_id]["obj"]
        translator = Translator(robot_name,
                               robot_id,
                               topic["msg_topic"],
                               topic_id,
                               obj,
                               node=node)
        translators.append(translator)
    
    return node


if __name__ == "__main__":
    # Initialize ROS2
    rclpy.init()
    
    # Create main node
    node = rclpy.create_node("topic_translator")
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
        # Create translator node and translators
        create_translator_node(this_robot, robot_configs_path, topic_configs_path, node)
        
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init(args=None)
    node = rclpy.create_node('translator')
    
    # Declare parameters
    node.declare_parameter('robot_name', '')
    node.declare_parameter('robot_configs', '')
    node.declare_parameter('topic_configs', '')
    
    logger = node.get_logger()
    
    # Get robot name from parameters 
    this_robot = node.get_parameter('robot_name').get_parameter_value().string_value
    if not this_robot:
        logger.error('robot_name parameter not set')
        sys.exit(1)

    # Get config file paths from parameters
    robot_configs_path = node.get_parameter("robot_configs").get_parameter_value().string_value
    topic_configs_path = node.get_parameter("topic_configs").get_parameter_value().string_value
    
    # Use empty strings as None for the function
    robot_configs_path = robot_configs_path if robot_configs_path else None
    topic_configs_path = topic_configs_path if topic_configs_path else None

    try:
        # Create translator node and translators
        create_translator_node(this_robot, robot_configs_path, topic_configs_path, node)
        
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
