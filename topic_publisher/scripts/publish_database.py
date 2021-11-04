#!/usr/bin/env python3

import os
import pdb
import sys

import rospkg
import rospy
import yaml

rospack = rospkg.RosPack()

comms_path = rospack.get_path("distributed_database")
scripts_path = os.path.join(comms_path, "scripts")
sys.path.append(scripts_path)

configs_path = rospack.get_path("network_configs")
robot_yaml_path = os.path.join(configs_path, "config", "robotConfigs.yml")

import database_server_utils as du
import distributed_database.srv
import nav_msgs.msg
import std_msgs.msg
from sensor_msgs.msg import CompressedImage, PointCloud2


class Map_Visualize:
    def __init__(self):
        rospy.init_node("basestation_pub", anonymous=True)
        self.select_service = "database_server/SelectDB"
        self.get_hash_service = "database_server/GetDataHashDB"

        self.select_db = rospy.ServiceProxy(
            self.select_service, distributed_database.srv.SelectDB
        )

        self.get_hash_db = rospy.ServiceProxy(
            self.get_hash_service, distributed_database.srv.GetDataHashDB
        )

        self.detection_topic_io = "/io/object_map_pc"
        self.odom_topic_io = "/io/odom_raw"

        self.detection_topic_callisto = "/callisto/object_map_pc"
        self.odom_topic_callisto = "/callisto/odom_raw"

        with open(robot_yaml_path, "r") as f:
            robot_cfg = yaml.load(f)
        self.robot_list = list(robot_cfg.keys())

        self.robot_list.remove("groundstation")
        # self.robot_list.remove("robot-io")
        # self.robot_list.remove("robot-callisto")

        self.map_pub = []

        self.detection_topic_io_pub = rospy.Publisher(
            self.detection_topic_io, PointCloud2, queue_size=10
        )

        self.detection_topic_io_pub_hash = rospy.Publisher(
            self.detection_topic_io + "_hash", std_msgs.msg.String, queue_size=10
        )

        self.odom_topic_io_pub = rospy.Publisher(
            self.odom_topic_io, nav_msgs.msg.Odometry, queue_size=10
        )

        self.odom_topic_io_pub_hash = rospy.Publisher(
            self.odom_topic_io + "_hash", std_msgs.msg.String, queue_size=10
        )
        self.detection_topic_callisto_pub = rospy.Publisher(
            self.detection_topic_callisto, PointCloud2, queue_size=10
        )

        self.detection_topic_callisto_pub_hash = rospy.Publisher(
            self.detection_topic_callisto + "_hash", std_msgs.msg.String, queue_size=10
        )

        self.odom_topic_callisto_pub = rospy.Publisher(
            self.odom_topic_callisto, nav_msgs.msg.Odometry, queue_size=10
        )

        self.odom_topic_callisto_pub_hash = rospy.Publisher(
            self.odom_topic_callisto + "_hash", std_msgs.msg.String, queue_size=10
        )

        rate = rospy.Rate(0.2)

        hashes = []

        while not rospy.is_shutdown():
            # Publish loop for all the nodes
            for j in range(len(self.robot_list)):

                hashes_to_get = []

                rospy.wait_for_service(self.select_service)

                try:
                    answ = self.select_db(self.robot_list[j], -1, -1)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))
                returned_hashes = answ.hashes

                for hash_ in returned_hashes:
                    if hash_ not in hashes:
                        hashes_to_get.append(hash_)

                for get_hash in hashes_to_get:
                    rospy.wait_for_service(self.get_hash_service)
                    try:
                        answ = self.get_hash_db(get_hash)
                        hashes.append(get_hash)
                    except rospy.ServiceException as exc:
                        print("Service did not process request: " + str(exc))
                        continue

                    ans_feat_name, ans_ts, ans_data, _ = du.parse_answer(answ)

                    print(40*"-", ans_feat_name)

                    # msg = nav_msgs.msg.Odometry(ans_data)
                    # pdb.set_trace()

                    if "robot-io-Detections" in ans_feat_name:
                        # print("published")
                        self.detection_topic_io_pub.publish(ans_data)
                        self.detection_topic_io_pub_hash.publish(get_hash)
                        print(len(hashes), ": IO-Detection")

                    elif "robot-io-Odom" in ans_feat_name:
                        # print("published")
                        self.odom_topic_io_pub.publish(ans_data)
                        self.odom_topic_io_pub_hash.publish(get_hash)
                        print(len(hashes), ": IO-Odom")

                    elif "robot-callisto-Detections" in ans_feat_name:
                        # print("published")
                        self.detection_topic_callisto_pub.publish(ans_data)
                        self.detection_topic_callisto_pub_hash.publish(get_hash)
                        print(len(hashes), ": Callisto-Detection")

                    elif "robot-callisto-Odom" in ans_feat_name:
                        # print("published")
                        self.odom_topic_callisto_pub.publish(ans_data)
                        self.odom_topic_callisto_pub_hash.publish(get_hash)
                        print(len(hashes), ": Callisto-Odom")
            rate.sleep()


if __name__ == "__main__":
    MV = Map_Visualize()
