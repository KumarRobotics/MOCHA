#!/usr/bin/env python3

import os
import pdb
import sys

import rospkg
import rospy
import yaml

from topic_publisher import TopicPublisher

if __name__ == "__main__":
    rospy.init_node("jackal_publisher", anonymous=True)

    list_of_topics =  ["/asoom/map_sem_img": sensor_msgs.msg.Image,
        "/asoom/map_sem_img_center": geometry_msgs.msg.PointStamped,
        "/asoom/recent_key_pose": geometry_msgs.msg.PoseStamped]

    pub = TopicPublisher(list_of_topics)
    pub.run()
