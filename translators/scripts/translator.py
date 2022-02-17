import os
import sys
import rospy
import rospkg
import distributed_database.srv

class Translator:
    def __init__(self, topic_name, msg_type):
        rospack = rospkg.RosPack()
        db_path = rospack.get_path("distributed_database")
        scripts_path = os.path.join(db_path, "scripts")
        sys.path.append(scripts_path)
        import database_server_utils as du
        self.__du = du

        self.__counter = 0
        self.__topic_name = topic_name
        self.__robot_name = du.get_robot_name("robotConfigs.yml")
        self.__service_name = "database_server/AddUpdateDB"
        self.__add_update_db = rospy.ServiceProxy(
                self.__service_name, distributed_database.srv.AddUpdateDB
        )

        rospy.Subscriber(
            self.__topic_name, msg_type, self.translator_cb
        )

    def translator_cb(self, data):
        msg = data

        feature_name = f"{self.__robot_name},{self.__topic_name},{self.__counter}"

        rospy.wait_for_service(self.__service_name)
        serialized_msg = self.__du.serialize_ros_msg(msg)
        try:
            answ = self.__add_update_db(feature_name, 
                    msg._md5sum, serialized_msg)
            answ_hash = answ.new_hash
            rospy.loginfo(f"Hash insert - {self.__topic_name} - {answ_hash}")
            self.__counter += 1
        except rospy.ServiceException as exc:
            rospy.logerr(f"Service did not process request: {exc}")



