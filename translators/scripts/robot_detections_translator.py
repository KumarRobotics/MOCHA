#!/usr/bin/env python3

import os
import sys

import distributed_database.srv
import nav_msgs.msg
import rospkg
import rospy


class OdomOnDetection:
    def __init__(self):
        self.odom_on_det_num = 1
        self.odom_on_det_topic = "/quadrotor/odom_on_detection"
        self.odom_on_det_name = "OdomOnDetection"
        self.add_service_name = "database_server/AddUpdateDB"
        self.add_update_db = rospy.ServiceProxy(
            self.add_service_name, distributed_database.srv.AddUpdateDB
        )

    def listen(self):
        rospy.init_node("odom_on_detection_listener", anonymous=True)
        rospy.Subscriber(
            self.odom_on_det_topic, nav_msgs.msg.Odometry, self.odom_on_det_cb
        )
        rospy.spin()

    def odom_on_det_cb(self, data):
        msg = data
        feature_name = f"{self.odom_on_det_name}-{self.odom_on_det_num}"

        rospy.wait_for_service(self.add_service_name)
        serialized_msg = du.serialize_ros_msg(msg)

        try:
            answ = self.add_update_db(feature_name, msg._md5sum, serialized_msg)
            answ_hash = answ.new_hash
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print("Got hash: " + answ_hash)

        self.odom_on_det_num += 1


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    db_path = rospack.get_path("distributed_database")
    scripts_path = os.path.join(db_path, "scripts")
    sys.path.append(scripts_path)

    import database_server_utils as du

    PI = OdomOnDetection()
    PI.listen()
