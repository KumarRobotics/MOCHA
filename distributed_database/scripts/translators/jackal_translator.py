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
    rospy.init_node("jackal_translator", anonymous=True)
    map_translator  = Translator("/object_mapper/map", sensor_msgs.msg.PointCloud2)
    pose_translator = Translator("/object_mapper/odom", nav_msgs.msg.Odometry)
    rospy.spin()
