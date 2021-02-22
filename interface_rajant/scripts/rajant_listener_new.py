#!/usr/bin/env python3

import ast
import yaml
import os
import re
import threading
import pprint
import sys
from matplotlib import pyplot as plt

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import rospkg

class RajantListener():

    def __init__(self, this_robot):

        self.MAC_DICT = {}
        self.this_robot = this_robot

        rospy.init_node('rajant_listener', anonymous=True)

        self.rate = rospy.Rate(10)
        self.cost = []

    def update_dict(self, data):

        fake_cost = -20
        dt = rospy.Duration(20.)

        alldata = data.data

        data_dict = ast.literal_eval(alldata)

        state = data_dict['watchResponse']['state']

        # Still need to figure out the rospy time issue
        for wireless_channel in state.keys():
            for wireless_keys in state[wireless_channel].keys():
                if wireless_keys[0:4] == 'peer':
                    peer = wireless_keys
                    if 'cost' in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        self.MAC_DICT[mac]['cost'] = state[wireless_channel][peer]['cost']
                        self.MAC_DICT[mac]['timestamp'] = rospy.Time.now()
                        self.cost.append(self.MAC_DICT[mac]['cost'])    
                    elif 'mac' in state[wireless_channel][peer].keys() and 'cost' not in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        self.cost.append(self.MAC_DICT[mac]['cost'])
        print(self.cost)

    def rajant_listener(self):

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('network_configs')
        # Get both config files
        radio_cfg_path = os.path.join(package_path, 'config', 'radioConfigs.yml')
        with open(radio_cfg_path, 'r') as f:
            self.radio_cfg = yaml.load(f)

        robot_cfg_path = os.path.join(package_path, 'config', 'robotConfigs.yml')
        with open(robot_cfg_path, 'r') as f:
            self.robot_cfg = yaml.load(f)

        fake_cost = -20

        for radio in self.radio_cfg.keys():
            for address in self.radio_cfg[radio]['MAC-address']:
                self.MAC_DICT[address] = {}
                self.MAC_DICT[address]['cost'] = fake_cost
                self.MAC_DICT[address]['timestamp'] = rospy.Time.now()
                self.MAC_DICT[address]['radio'] = radio

        while not rospy.is_shutdown():
    
            rospy.Subscriber('rajant_log/log', String, self.update_dict)

if __name__ == '__main__':

    rospack = rospkg.RosPack()
    comms_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(comms_path, 'scripts')
    sys.path.append(scripts_path)
    import database_server as ds

    this_robot = ds.get_robot_name()
    RL = RajantListener(this_robot)
    RL.rajant_listener()

