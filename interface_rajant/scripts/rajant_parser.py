#!/usr/bin/env python3

import ast
import yaml
import os
import re
import threading
import pprint
import sys
import pdb

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory

class RajantParser(Node):
    def __init__(self, this_robot, robot_configs, radio_configs):
        super().__init__('rajant_parser')

        # Check input args
        assert isinstance(this_robot, str)
        assert isinstance(robot_configs, dict)
        assert isinstance(radio_configs, dict)
        self.MAC_DICT = {}

        self.this_robot = this_robot
        self.robot_cfg = robot_configs
        self.radio_cfg = radio_configs

        self.get_logger().info(f"{self.this_robot} - Rajant API Parser - Starting")

        # Generate a standard configuration with a RSSI of -1
        for radio in self.radio_cfg.keys():
            for address in self.radio_cfg[radio]['MAC-address']:
                self.MAC_DICT[address] = {}
                self.MAC_DICT[address]['rssi'] = -20
                self.MAC_DICT[address]['timestamp'] = self.get_clock().now()
                self.MAC_DICT[address]['radio'] = radio
                self.MAC_DICT[address]['publisher'] = None

        # Generate publishers for each item in the dict
        for mac in self.MAC_DICT.keys():
            for robot in self.robot_cfg.keys():
                if self.MAC_DICT[mac]['radio'] == self.robot_cfg[robot]['using-radio'] and robot != self.this_robot:
                    self.MAC_DICT[mac]['publisher'] = self.create_publisher(Int32, 'mocha/rajant/rssi/' + robot, 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'mocha/rajant/log',
            self.update_dict,
            10
        )


    def update_dict(self, data):
        # If we did not receive an update after dt, drop the RSSI to -1
        no_rssi = -1
        dt = rclpy.duration.Duration(seconds=20.0)

        # Evaluate the input data as a dictionary
        alldata = data.data
        data_dict = ast.literal_eval(data.data)

        state = data_dict['watchResponse']['state']

        # Update the RSSI
        for wireless_channel in state.keys():
            for wireless_keys in state[wireless_channel].keys():
                if wireless_keys[0:4] == 'peer':
                    peer = wireless_keys
                    if 'rssi' in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        if mac not in self.MAC_DICT.keys():
                            self.get_logger().error(f"MAC: {mac} is not in the list of knowns MACs. Is your radio_configs.yaml file correct?")
                            continue
                        rssi = state[wireless_channel][peer]['rssi']
                        self.MAC_DICT[mac]['rssi'] = rssi
                        self.MAC_DICT[mac]['timestamp'] = self.get_clock().now()
                        # Only publish if the publisher is not None
                        # This avoids an error for a radio that is connected but that is not
                        # actively used by any robot
                        if self.MAC_DICT[mac]['publisher'] is not None:
                            msg = Int32()
                            msg.data = rssi
                            self.MAC_DICT[mac]['publisher'].publish(msg)
                        else:
                            self.get_logger().warn(f"{self.this_robot} - Rajant API Parser - " +
                                          f"active radio {self.MAC_DICT[mac]['radio']} not assigned to any robot")
                    elif 'mac' in state[wireless_channel][peer].keys() and 'rssi' not in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        if mac not in self.MAC_DICT.keys():
                            self.get_logger().error(f"MAC: {mac} is not in the list of knowns MACs. Is your radio_configs.yaml file correct?")
                            continue
                        if self.get_clock().now() - self.MAC_DICT[mac]['timestamp'] > dt:
                            self.MAC_DICT[mac]['rssi'] = no_rssi
                            # Only publish if the publisher is not None
                            # See comment above
                            if self.MAC_DICT[mac]['publisher'] is not None:
                                msg = Int32()
                                msg.data = no_rssi
                                self.MAC_DICT[mac]['publisher'].publish(msg)
                            else:
                                self.get_logger().warn(f"{self.this_robot} - Rajant API Parser - " +
                                              f"active radio {self.MAC_DICT[mac]['radio']} not assigned to any robot")


def main(args=None):
    rclpy.init(args=args)

    # Create a temporary node to get parameters
    temp_node = Node('temp_rajant_parser')

    # Declare parameters
    temp_node.declare_parameter('robot_name', 'charon')
    temp_node.declare_parameter('robot_configs', '')
    temp_node.declare_parameter('radio_configs', '')

    # Get parameters
    robot_name = temp_node.get_parameter('robot_name').get_parameter_value().string_value
    robot_configs_file = temp_node.get_parameter('robot_configs').get_parameter_value().string_value
    radio_configs_file = temp_node.get_parameter('radio_configs').get_parameter_value().string_value

    # Load robot configs
    with open(robot_configs_file, "r") as f:
        robot_configs = yaml.load(f, Loader=yaml.FullLoader)
    if robot_name not in robot_configs.keys():
        temp_node.get_logger().error("Robot not in config file")
        temp_node.destroy_node()
        rclpy.shutdown()
        return

    # Load radio configs
    with open(radio_configs_file, "r") as f:
        radio_configs = yaml.load(f, Loader=yaml.FullLoader)
    radio = robot_configs[robot_name]["using-radio"]
    if radio not in radio_configs.keys():
        temp_node.get_logger().error("Radio not in config file")
        temp_node.destroy_node()
        rclpy.shutdown()
        return

    # Clean up temp node
    temp_node.destroy_node()

    # Create the actual parser node
    rajant_parser = RajantParser(robot_name, robot_configs, radio_configs)

    try:
        rclpy.spin(rajant_parser)
    except KeyboardInterrupt:
        pass
    finally:
        rajant_parser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
