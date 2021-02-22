#!/usr/bin/env python3

import os
import sys

import rospy
import rospkg

import nav_msgs.msg

import distributed_database.srv

class RobotPose():
    def __init__(self):
        self.robot_pose_num = 1
        self.robot_pose_topic = '/quadrotor/odom_throttled_low'
        self.robot_pose_name = 'OdomThrottled'
        self.add_service_name = 'database_server/AddUpdateDB'
        self.add_update_db = rospy.ServiceProxy(self.add_service_name, distributed_database.srv.AddUpdateDB)

    def listen(self):
        rospy.init_node('odom_throttled_listener', anonymous=True)
        rospy.Subscriber(self.robot_pose_topic, nav_msgs.msg.Odometry, self.robot_pose_cb)
        rospy.spin()

    def robot_pose_cb(self, data):
        msg = data
        feature_name = f"{self.robot_pose_name}-{self.robot_pose_num}"

        rospy.wait_for_service(self.add_service_name)
        serialized_msg = du.serialize_ros_msg(msg)

        try:
            answ = self.add_update_db(feature_name, msg._md5sum, serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print('Service did not process request: ' + str(exc))
        print('Got hash: ' + answ_hash)

        self.robot_pose_num += 1

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    db_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(db_path, 'scripts')
    sys.path.append(scripts_path)

    import database_server_utils as du

    PI = RobotPose()
    PI.listen()


