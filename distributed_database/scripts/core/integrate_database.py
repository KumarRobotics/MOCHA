#!/usr/bin/env python3
import os
import random
import signal
import sys
import time
import traceback
from functools import partial

import distributed_database.msg
import distributed_database.srv
import roslaunch
import rospkg
import rospy
import yaml
from std_msgs.msg import Int32

import database_server as ds
import database_server_utils as du
import synchronize_channel as sync


class Integrate:
    def __init__(self, config_file, this_robot):

        self.this_robot = this_robot

        self.config_file = config_file

        self.DBServer = ds.DatabaseServer(self.config_file)

        self.comm_nodes = []
        self.other_robots = []
        self.num_robot_in_comm = 0

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("distributed_database")
        robot_yaml_path = os.path.join(package_path, "config", config_file)
        with open(robot_yaml_path, "r") as f:
            robot_cfg = yaml.load(f)
        robot_list = robot_cfg.keys()

        for target_robot in robot_list:
            if target_robot not in robot_cfg[self.this_robot]["clients"].keys():
                rospy.logwarn(
                    f'Skipping target:"{target_robot}" as it is not in the graph of this robot'
                )
                continue

            if robot_cfg[target_robot]["is-active"] and target_robot != self.this_robot:
                node = sync.Channel(
                    self.DBServer.dbl, self.this_robot, target_robot, "robotConfigs.yml"
                )
                self.comm_nodes.append(node)

                self.other_robots.append(target_robot)
                node.run()
                try:
                    time.sleep(2)
                except KeyboardInterrupt:
                    print("INTEGRATE: Killed while initializing (cleaning up nodes)")
                    for node in self.comm_nodes:
                        node.comm_node.terminate()
                    return

        print("INTEGRATE: Created all nodes!")
        self.listen()

    def comms_callback(self, data, comm_node):

        rssi = data.data

        if rssi > 20:
            self.num_robot_in_comm += 1
            try:
                comm_node.trigger_sync()
                time.sleep(10)
            except:
                traceback.print_exception(*sys.exc_info())

    def timer_cb(self, robot_id, event):
        try:
            self.comm_nodes[robot_id].trigger_sync()
        except:
            rospy.logwarn(f"{self.comm_nodes[robot_id].robot} - {self.comm_nodes[robot_id].target_robot} trigger failed. Will try again soon")
            rospy.sleep(random.random() * 15)

    def listen(self):

        rospy.init_node("integrate_database", anonymous=True)
        num_robots = len(self.other_robots)
        self.interrupted = False

        def signal_handler(sig, frame):
            print("INTEGRATE: Got signal (killing comm nodes)")
            self.interrupted = True
            for node in self.comm_nodes:
                node.comm_node.terminate()

        signal.signal(signal.SIGINT, signal_handler)

        for _ in range(30):
            if self.interrupted:
                return
            rospy.sleep(1)

        print("INTEGRATE: entering main loop")

        # Start timers to sync with all the robots
        # timers = []
        # for i in range(num_robots):
        #     timers.append(rospy.Timer(rospy.Duration(5), partial(self.timer_cb, i)))

        for i in range(num_robots):
            rospy.Subscriber('rajant_log/log/' + self.this_robot + '/' + self.other_robots[i], Int32, self.comms_callback, self.comm_nodes[i])

        rospy.spin()

if __name__ == "__main__":
    CONFIG_FILE = "robotConfigs.yml"
    this_robot = du.get_robot_name(CONFIG_FILE)
    print(this_robot)

    InAll = Integrate(CONFIG_FILE, this_robot)
