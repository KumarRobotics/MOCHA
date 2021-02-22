#!/usr/bin/env python3

import ast
import yaml
import os
import re
import threading
import pprint
import sys

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import rospkg

CONFIG_FILE = "robotConfigs.yml"

class RajantListener():

    def __init__(self, this_robot):

        self.MAC_DICT = {}
        self.this_robot = this_robot

        rospy.init_node('rajant_listener', anonymous=True)

        self.rate = rospy.Rate(.5)

    def generate_publishers(self):

        for mac in self.MAC_DICT.keys():
            for robot in self.robot_cfg.keys():
                if self.MAC_DICT[mac]['radio'] == self.robot_cfg[robot]['using-radio'] and robot != self.this_robot:
                    self.MAC_DICT[mac]['publisher'] = rospy.Publisher('rajant_log/log/' + self.this_robot + '/' + robot, Int32, queue_size = 10)

        return self.MAC_DICT

    def update_dict(self, data):

        fake_rssi = -20
        dt = rospy.Duration(20.)

        alldata = data.data

        data_dict = ast.literal_eval(alldata)

        state = data_dict['watchResponse']['state']

        # Still need to figure out the rospy time issue
        for wireless_channel in state.keys():
            for wireless_keys in state[wireless_channel].keys():
                if wireless_keys[0:4] == 'peer':
                    peer = wireless_keys
                    if 'rssi' in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        self.MAC_DICT[mac]['rssi'] = state[wireless_channel][peer]['rssi']
                        self.MAC_DICT[mac]['timestamp'] = rospy.Time.now()
                    elif 'mac' in state[wireless_channel][peer].keys() and 'rssi' not in state[wireless_channel][peer].keys():
                        mac = state[wireless_channel][peer]['mac']
                        if rospy.Time.now()-self.MAC_DICT[mac]['timestamp'] > dt:
                            self.MAC_DICT[mac]['rssi'] = fake_rssi

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

        fake_rssi = -20

        for radio in self.radio_cfg.keys():
            for address in self.radio_cfg[radio]['MAC-address']:
                self.MAC_DICT[address] = {}
                self.MAC_DICT[address]['rssi'] = fake_rssi
                self.MAC_DICT[address]['timestamp'] = rospy.Time.now()
                self.MAC_DICT[address]['radio'] = radio

        self.MAC_DICT = self.generate_publishers()

        rospy.Subscriber('rajant_log/log', String, self.update_dict)

        while not rospy.is_shutdown():
            self.rate.sleep()
            for mac in self.MAC_DICT.keys():
                if 'publisher' in self.MAC_DICT[mac].keys():
                    rospy.loginfo(self.MAC_DICT[mac]['rssi'])
                    self.MAC_DICT[mac]['publisher'].publish(self.MAC_DICT[mac]['rssi'])

                    #rospy.sleep(0.1)

if __name__ == '__main__':

    rospack = rospkg.RosPack()
    comms_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(comms_path, 'scripts')
    sys.path.append(scripts_path)
    import database_server as ds
    import database_server_utils as du

    this_robot = du.get_robot_name(CONFIG_FILE)
    RL = RajantListener(this_robot)
    RL.rajant_listener()

