#!/usr/bin/env python3

from topic_publisher import TopicPublisher
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import rospy

if __name__ == "__main__":
    rospy.init_node("jackal_publisher", anonymous=True)

    list_of_topics =  {
        "callisto,/object_mapper/map": sensor_msgs.msg.PointCloud2,
        "callisto,/object_mapper/odom": geometry_msgs.msg.PoseStamped,
        "europa,/object_mapper/map": sensor_msgs.msg.PointCloud2,
        "europa,/object_mapper/odom": geometry_msgs.msg.PoseStamped,
        }

    pub = TopicPublisher(list_of_topics)
    pub.run()
