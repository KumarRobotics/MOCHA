#!/usr/bin/env python3

import ast
import yaml
import os
import re
import threading
import pprint
import sys
import pdb

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import rospkg

class RajantParser():
    def __init__(self, this_robot, robot_configs, radio_configs):

        # Check input args
        assert isinstance(this_robot, str)
        assert isinstance(robot_configs, dict)
        assert isinstance(radio_configs, dict)
        self.MAC_DICT = {}

        self.this_robot = this_robot
        self.robot_cfg = robot_configs
        self.radio_cfg = radio_configs
        self.rate = rospy.Rate(.5)

        rospy.loginfo(f"{self.this_robot} - Rajant API Parser - Starting")

        # Generate a standard configuration with a RSSI of -1
        for radio in self.radio_cfg.keys():
            for address in self.radio_cfg[radio]['MAC-address']:
                self.MAC_DICT[address] = {}
                self.MAC_DICT[address]['rssi'] = -20
                self.MAC_DICT[address]['timestamp'] = rospy.Time.now()
                self.MAC_DICT[address]['radio'] = radio
                self.MAC_DICT[address]['publisher'] = None

        # Generate publishers for each item in the dict
        for mac in self.MAC_DICT.keys():
            for robot in self.robot_cfg.keys():
                if self.MAC_DICT[mac]['radio'] == self.robot_cfg[robot]['using-radio'] and robot != self.this_robot:
                    self.MAC_DICT[mac]['publisher'] = rospy.Publisher('ddb/rajant/rssi/' + robot, Int32, queue_size = 10)

        rospy.Subscriber('ddb/rajant/log', String, self.update_dict)
        rospy.spin()


    def update_dict(self, data):
        # If we did not receive an update after dt, drop the RSSI to -1
        no_rssi = -1
        dt = rospy.Duration(20.)

        # Evaluate the input data as a dictionary
        alldata = data.data
        data_dict = ast.literal_eval(data.data)

        state = data_dict['watchResponse']['state']

        # Still need to figure out the rospy time issue
        # Update the RSSI
        for wireless_channel in state.keys():
            for wireless_keys in state[wireless_channel].keys():
                if wireless_keys[0:4] == 'peer':
                    peer = wireless_keys
                    if 'rssi' in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        if mac not in self.MAC_DICT.keys():
                            rospy.logerr(f"MAC: {mac} is not in the list of knowns MACs. Is your radio_configs.yaml file correct?")
                            continue
                        rssi =  state[wireless_channel][peer]['rssi']
                        self.MAC_DICT[mac]['rssi'] = rssi
                        self.MAC_DICT[mac]['timestamp'] = rospy.Time.now()
                        # Only publish if the publisher is not None
                        # This avoids an error for a radio that is connected but that is not
                        # actively used by any robot
                        if self.MAC_DICT[mac]['publisher'] is not None:
                            self.MAC_DICT[mac]['publisher'].publish(rssi)
                        else:
                            rospy.logwarn(f"{self.this_robot} - Rajant API Parser - " +
                                          f"active radio {self.MAC_DICT[mac]['radio']} not assigned to any robot")
                    elif 'mac' in state[wireless_channel][peer].keys() and 'rssi' not in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        if mac not in self.MAC_DICT.keys():
                            rospy.logerr(f"MAC: {mac} is not in the list of knowns MACs. Is your radio_configs.yaml file correct?")
                            continue
                        if rospy.Time.now()-self.MAC_DICT[mac]['timestamp'] > dt:
                            self.MAC_DICT[mac]['rssi'] = no_rssi
                            # Only publish if the publisher is not None
                            # See comment above
                            if self.MAC_DICT[mac]['publisher'] is not None:
                                self.MAC_DICT[mac]['publisher'].publish(no_rssi)
                            else:
                                rospy.logwarn(f"{self.this_robot} - Rajant API Parser - " +
                                              f"active radio {self.MAC_DICT[mac]['radio']} not assigned to any robot")


if __name__ == '__main__':

    rospy.init_node('rajant_listener', anonymous=False)
    # Get robot name from the ~robot_name param
    robot_name = rospy.get_param('~robot_name', 'charon')

    # Get robot configs
    robot_configs_file = rospy.get_param("~robot_configs")
    with open(robot_configs_file, "r") as f:
       robot_configs = yaml.load(f, Loader=yaml.FullLoader)
    if robot_name not in robot_configs.keys():
        rospy.signal_shutdown("Robot not in config file")
        rospy.spin()

    # Get radio configs
    radio_configs_file = rospy.get_param("~radio_configs")
    with open(radio_configs_file, "r") as f:
        radio_configs = yaml.load(f, Loader=yaml.FullLoader)
    radio = robot_configs[robot_name]["using-radio"]
    if radio not in radio_configs.keys():
        rospy.shutdown("Radio not in config file")
        rospy.spin()

    RajantParser(robot_name, robot_configs, radio_configs)
