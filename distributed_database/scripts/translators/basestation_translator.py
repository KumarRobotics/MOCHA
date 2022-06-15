#!/usr/bin/env python3

import os
import sys

import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg

from translator import Translator


if __name__ == "__main__":
    rospy.init_node("basestation_translator", anonymous=True)
    goal_translator = Translator("/target_goals", geometry_msgs.msg.PoseArray)
    rospy.spin()
