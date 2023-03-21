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

class RajantListener():
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

        rospy.init_node('rajant_listener', anonymous=False)

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

if __name__ == '__main__':

    rospack = rospkg.RosPack()
    comms_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(comms_path, 'scripts')
    sys.path.append(scripts_path)

    # Get robot name from the ~robot_name param
    robot_name = rospy.get_param('~robot_name', 'charon')

    # Get robot configs
    robot_configs_file = rospy.get_param("~robot_configs")
    with open(robot_configs_file, "r") as f:
       robot_configs = yaml.load(f, Loader=yaml.FullLoader)
   if robot_name not in robot_configs.keys():
       rospy.shutdown("Robot not in config file")
       rospy.spin()

    # Get radio configs
    radio_configs_file = rospy.get_param("~radio_configs")
    with open(radio_configs_file, "r") as f:
        radio_configs = yaml.load(f, Loader=yaml.FullLoader)
    radio = robot_configs[this_robot]["using-radio"]
    if radio not in radio_configs.keys():
        rospy.shutdown("Radio not in config file")
        rospy.spin()

    RL = RajantListener(robot_name, robot_configs, radio_configs)
    RL.rajant_listener()
