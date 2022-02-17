#!/usr/bin/env python3

import os
import sys

import rospy
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

from translator import Translator


if __name__ == "__main__":
    rospy.init_node("map_listener", anonymous=True)
    map_translator = Translator("/asoom/map_sem_img", sensor_msgs.msg.Image)
    map_center_translator = Translator("/asoom/map_sem_img_center", geometry_msgs.msg.PointStamped)
    recent_key_pose_translator = Translator("/asoom/recent_key_pose", geometry_msgs.msg.PoseStamped)
    rospy.spin()
