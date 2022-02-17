#!/usr/bin/env python3

import os
import sys

import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg

from translator import Translator


if __name__ == "__main__":
    rospy.init_node("jackal_translator", anonymous=True)
    path_translator = Translator("/semantic_planner/path", nav_msgs.msg.Path)
    pose_translator = Translator("/upslam/global_pose", geometry_msgs.msg.PoseStamped)
    rospy.spin()
