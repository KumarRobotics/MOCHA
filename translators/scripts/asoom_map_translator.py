#!/usr/bin/env python3

import os
import sys

from trasnlator import Translator
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg


if __name__ == "__main__":
    rospy.init_node("map_listener", anonymous=True)
    map_translator = Traslator("/asoom/map_sem_img")
    map_center_translator = Translator("/asoom/map_sem_img_center")
    recent_key_pose_translator = Translator("/asoom/recent_key_pose")
    rospy.spin()
