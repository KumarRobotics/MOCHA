#!/usr/bin/env python3

import os
import sys

import distributed_database.srv
import nav_msgs.msg
import rospkg
import rospy
import sensor_msgs.msg
import geometry_msgs.msg


class Map:
    def __init__(self):
        self.map_num = 1
        self.map_topic = "/asoom/map_sem_img"
        self.map_name = "map_image"
        self.add_service_name = "database_server/AddUpdateDB"
        self.robot_name = du.get_robot_name("robotConfigs.yml")
        self.add_update_db = rospy.ServiceProxy(
            self.add_service_name, distributed_database.srv.AddUpdateDB
        )

    def listen(self):
        rospy.Subscriber(
            self.map_topic, sensor_msgs.msg.Image, self.map_cb
        )

    def map_cb(self, data):
        msg = data
        feature_name = f"{self.robot_name}-{self.map_name}-{self.map_num}"

        rospy.wait_for_service(self.add_service_name)
        serialized_msg = du.serialize_ros_msg(msg)

        try:
            answ = self.add_update_db(feature_name, msg._md5sum, serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print("Got hash: " + answ_hash)

        self.map_num += 1

class Map_center:
    def __init__(self):
        self.map_center_num = 1
        self.map_center_topic = "/asoom/map_sem_img_center"
        self.map_center_name = "map_center"
        self.add_service_name = "database_server/AddUpdateDB"
        self.robot_name = du.get_robot_name("robotConfigs.yml")
        self.add_update_db = rospy.ServiceProxy(
            self.add_service_name, distributed_database.srv.AddUpdateDB
        )

    def listen(self):
        rospy.Subscriber(
            self.map_center_topic, geometry_msgs.msg.PointStamped, self.map_center_cb
        )

    def map_center_cb(self, data):
        msg = data
        feature_name = f"{self.robot_name}-{self.map_center_name}-{self.map_center_num}"

        rospy.wait_for_service(self.add_service_name)
        serialized_msg = du.serialize_ros_msg(msg)

        try:
            answ = self.add_update_db(feature_name, msg._md5sum, serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print("Got hash: " + answ_hash)

        self.map_center_num += 1

class RKP:
    def __init__(self):
        self.recent_key_pose_num = 1
        self.recent_key_pose_topic = "/asoom/recent_key_pose"
        self.recent_key_pose_name = "recent_key_pose"
        self.add_service_name = "database_server/AddUpdateDB"
        self.robot_name = du.get_robot_name("robotConfigs.yml")
        self.add_update_db = rospy.ServiceProxy(
            self.add_service_name, distributed_database.srv.AddUpdateDB
        )

    def listen(self):
        rospy.Subscriber(
            self.recent_key_pose_topic, geometry_msgs.msg.PoseStamped, self.recent_key_pose_cb
        )

    def recent_key_pose_cb(self, data):
        msg = data
        feature_name = f"{self.robot_name}-{self.recent_key_pose_name}-{self.recent_key_pose_num}"

        rospy.wait_for_service(self.add_service_name)
        serialized_msg = du.serialize_ros_msg(msg)

        try:
            answ = self.add_update_db(feature_name, msg._md5sum, serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print("Got hash: " + answ_hash)

        self.recent_key_pose_num += 1


if __name__ == "__main__":
    rospy.init_node("map_listener", anonymous=True)
    rospack = rospkg.RosPack()
    db_path = rospack.get_path("distributed_database")
    scripts_path = os.path.join(db_path, "scripts")
    sys.path.append(scripts_path)

    import database_server_utils as du

    PI = Map()
    PI.listen()
    MC = Map_center()
    MC.listen()
    KP = RKP()
    KP.listen()
    rospy.spin()
