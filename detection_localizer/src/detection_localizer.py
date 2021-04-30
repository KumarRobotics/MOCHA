#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry

class DetectionLocalizer:
    def __init__(self):
        self.last_pose_ = None
        self.last_det_loc_ = None
        self.pose_sub_ = rospy.Subscriber('/upslam/global_pose', PoseStamped, self.pose_cb)
        self.detection_sub_ = rospy.Subscriber('/detection_proc/detections', Detection2DArray, self.det_cb)
        self.pose_heartbeat_pub_ = rospy.Publisher('/robot/odom', Odometry, queue_size=1)
        self.detection_pose_pub_ = rospy.Publisher('/robot/det_odom', Odometry, queue_size=1)

        self.heartbeat_timer_ = rospy.Timer(rospy.Duration(1), self.heartbeat_cb)

    def heartbeat_cb(self, timer):
        if self.last_pose_ is not None:
            if (self.last_pose_ - rospy.Time.now()).to_sec() < 1:
                odom = Odometry()
                odom.header = self.last_pose_.header
                odom.child_frame_id = 'robot'
                odom.pose.pose = self.last_pose_.pose
                self.pose_heartbeat_pub_.publish(odom)
            else:
                rospy.logwarn('Odometry out of date')
    
    def pose_cb(self, pose_msg):
        self.last_pose_ = pose_msg

    def report_det(self, det_loc):
        if self.last_pose_ is not None:
            odom = Odometry()
            odom.header = self.last_pose_.header
            odom.child_frame_id = 'detection'
            odom.pose.pose = self.last_pose_.pose
            odom.pose.pose.orientation.x = 0
            odom.pose.pose.orientation.y = 0
            odom.pose.pose.orientation.z = 0
            odom.pose.pose.orientation.w = 1
            self.detection_pose_pub_.publish(odom)

    def det_cb(self, det_msg):
        if self.last_pose_ is not None:
            det_loc = np.array([self.last_pose_.pose.position.x, self.last_pose_.pose.position.y])
            if self.last_det_loc_ is None:
                self.report_det(det_loc)
            elif np.linalg.norm(self.last_det_loc_ - det_loc) > 5:
                self.report_det(det_loc)

if __name__=='__main__':
    rospy.init_node('detection_localizer')
    dl = DetectionLocalizer()
    rospy.spin()
