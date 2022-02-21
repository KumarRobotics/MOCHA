#!/usr/bin/env python3

from topic_publisher import TopicPublisher
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import rospy

if __name__ == "__main__":
    rospy.init_node("jackal_publisher", anonymous=True)

    list_of_topics =  {"iapetus,/asoom/map_sem_img": sensor_msgs.msg.Image,
        "iapetus,/asoom/map_sem_img_center": geometry_msgs.msg.PointStamped,
        "iapetus,/asoom/recent_key_pose": geometry_msgs.msg.PoseStamped,
        "callisto,/semantic_planner/path": nav_msgs.msg.Path,
        "callisto,/upslam/global_pose_throttled": geometry_msgs.msg.PoseStamped,
        "callisto,/semantic_planner/target_goals": geometry_msgs.msg.PoseArray,
        "europa,/semantic_planner/path": nav_msgs.msg.Path,
        "europa,/upslam/global_pose_throttled": geometry_msgs.msg.PoseStamped,
        "europa,/semantic_planner/target_goals": geometry_msgs.msg.PoseArray,
        }

    pub = TopicPublisher(list_of_topics)
    pub.run()
