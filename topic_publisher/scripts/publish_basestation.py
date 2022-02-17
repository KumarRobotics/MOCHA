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
import geometry_msgs.msg
import sensor_msgs.msg


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

        self.map_sem_img_iapetus = "/iapetus/asoom/map_sem_img"
        self.map_sem_center_iapetus = "/iapetus/asoom/map_sem_img_center"
        self.recent_key_pose_iapetus = "/iapetus/asoom/recent_key_pose"

        with open(robot_yaml_path, "r") as f:
            robot_cfg = yaml.load(f)
        self.robot_list = list(robot_cfg.keys())

        self.robot_list.remove("groundstation")
        # self.robot_list.remove("robot-io")
        # self.robot_list.remove("robot-callisto")

        self.map_pub = []

        self.map_sem_img_iapetus_pub = rospy.Publisher(
            self.map_sem_img_iapetus, sensor_msgs.msg.Image, queue_size=10
        )

        self.map_sem_img_iapetus_pub_hash = rospy.Publisher(
            self.map_sem_img_iapetus + "_hash", std_msgs.msg.String, queue_size=10
        )

        self.map_sem_center_iapetus_pub = rospy.Publisher(
            self.map_sem_center_iapetus, geometry_msgs.msg.PointStamped, queue_size=10
        )

        self.map_sem_center_iapetus_pub_hash = rospy.Publisher(
            self.map_sem_center_iapetus + "_hash", std_msgs.msg.String, queue_size=10
        )
        self.recent_key_pose_iapetus_pub = rospy.Publisher(
            self.recent_key_pose_iapetus, geometry_msgs.msg.PoseStamped, queue_size=10
        )

        self.recent_key_pose_iapetus_pub_hash = rospy.Publisher(
            self.recent_key_pose_iapetus + "_hash", std_msgs.msg.String, queue_size=10
        )

        rate = rospy.Rate(.2)

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

                    if "robot-iapetus-map_image" in ans_feat_name:
                        # print("published")
                        self.map_sem_img_iapetus_pub.publish(ans_data)
                        self.map_sem_img_iapetus_pub_hash.publish(get_hash)
                        print(len(hashes), ": Iapetus-img")

                    elif "robot-iapetus-map_center" in ans_feat_name:
                        # print("published")
                        self.map_sem_center_iapetus_pub.publish(ans_data)
                        self.map_sem_center_iapetus_pub_hash.publish(get_hash)
                        print(len(hashes), ": Iapetus-center")

                    elif "robot-iapetus-recent_key_pose" in ans_feat_name:
                        # print("published")
                        self.recent_key_pose_iapetus_pub.publish(ans_data)
                        self.recent_key_pose_iapetus_pub_hash.publish(get_hash)
                        print(len(hashes), ": Iapetus-key-pose")
            rate.sleep()


if __name__ == "__main__":
    MV = Map_Visualize()
