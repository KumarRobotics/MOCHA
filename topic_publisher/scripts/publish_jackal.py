#!/usr/bin/env python3

from topic_publisher import TopicPublisher
import sensor_msgs.msg
import geometry_msgs.msg
import rospy

if __name__ == "__main__":
    rospy.init_node("jackal_publisher", anonymous=True)

    list_of_topics =  {"iapetus,/asoom/map_sem_img": sensor_msgs.msg.Image,
        "iapetus,/asoom/map_sem_img_center": geometry_msgs.msg.PointStamped,
        "iapetus,/asoom/recent_key_pose": geometry_msgs.msg.PoseStamped}

    pub = TopicPublisher(list_of_topics)
    pub.run()
