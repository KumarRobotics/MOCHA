#!/usr/bin/env python3

import os
import threading
import rospy
import distributed_database.srv
import database
import pdb
import database_utils as du
import importlib

class DatabaseServer:
    """ Starts a clean database and offers an API-like service
    to interact with the databaseself.

    Please see the list of services in the srv folder
    """
    def __init__(self, robot_configs, topic_configs):
        # Check input topics
        assert isinstance(robot_configs, dict)
        assert isinstance(topic_configs, dict)

        self.robot_configs = robot_configs
        self.topic_configs = topic_configs

        # Get current robot name from params
        self.robot_name = rospy.get_param('~robot_name')

        # Create the empty database with lock
        self.dbl = database.DBwLock()

        # Get current robot number
        self.robot_number = du.get_robot_number(self.robot_configs,
                                                robot_name=self.robot_name)

        # create services for all the possible calls to the DB
        self.service_list = []
        s = rospy.Service('~AddUpdateDB',
                          distributed_database.srv.AddUpdateDB,
                          self.add_update_db_service_cb)
        self.service_list.append(s)
        s = rospy.Service('~GetDataHashDB',
                          distributed_database.srv.GetDataHashDB,
                          self.get_data_hash_db_service_cb)
        self.service_list.append(s)
        s = rospy.Service('~SelectDB',
                          distributed_database.srv.SelectDB,
                          self.select_db_service_cb)
        self.service_list.append(s)
        s = rospy.Service('~SetAckBitDB',
                          distributed_database.srv.SetAckBitDB,
                          self.set_ack_bit_cb)
        self.service_list.append(s)

        # Build the message_type object from the topic_configs file
        msg_list = []
        for robot in topic_configs:
            for topic in topic_configs[robot]:
                msg = topic['msg_type']
                # Check if the message is a valid one: it has only two parts
                # and all the characters are alphanumeric or _
                parts = msg.split('/')
                if not (len(parts) == 2 and
                        all(part.replace("_", "").isalnum()
                            for part in parts)):
                    rospy.logerr(f"Error: msg_type {msg} not valid")
                    self.shutdown()
                    rospy.signal_shutdown("Error: msg_type {msg} not valid")
                    rospy.spin()
                msg_list.append(topic['msg_type'])

        self.msg_types = {}
        for i, msg in enumerate(set(msg_list)):
            package_name, msg_name = msg.split('/')
            package = importlib.import_module(package_name + '.msg')
            message_type = getattr(package, msg_name)
            self.msg_types[message_type._md5sum] = {"dtype": i,
                                                    "obj": message_type,
                                                    "name": msg}
        # Expand the topic_configs message to include the pointer to the object,
        # and translate the priority to a numberic value
        # for robot in topic_configs:
        #     for topic in topic_configs[robot]:
        #         topic['msg_type_obj'] = imported_msgs[topic["msg_type"]]
        #         if topic["msg_priority"] == "NO_PRIORITY":
        #             topic["msg_priority_num"] = 0
        #         elif topic["msg_priority"] == "LOW_PRIORITY":
        #             topic["msg_priority_num"] = 1
        #         elif topic["msg_priority"] == "MEDIUM_PRIORITY":
        #             topic["msg_priority_num"] = 2
        #         elif topic["msg_priority"] == "HIGH_PRIORITY":
        #             topic["msg_priority_num"] = 3
        #         else:
        #             rospy.logerr(f"Error: msg_prio {topic['msg_priority']}" +
        #                          " not valid")
        #             rospy.signal_shutdown("Error: msg_priority not valid")
        #             rospy.spin()

    def add_update_db_service_cb(self, req):
        curr_time = rospy.get_time()
        if not len(req.msg_name):
            rospy.logerr("Error: msg_name empty")
            return
        dbm = database.DBMessage(self.robot_number,
                                 topic_name=req.msg_name,
                                 dtype=self.msg_types[req.msg_type_hash]['dtype'],
                                 priority=req.priority,
                                 ts=curr_time,
                                 data=req.msg_content,
                                 ack=False)

        checksum = self.dbl.add_modify_data(dbm)
        return distributed_database.srv.AddUpdateDBResponse(checksum)

    def get_data_hash_db_service_cb(self, req):
        if not len(req.msg_hash):
            rospy.logerr("Error: msg_hash empty")
            self.shutdown()
            rospy.signal_shutdown("Error: msg_hash empty")
            rospy.spin()
        dbm = self.dbl.find_hash(req.msg_hash)
        # Find key for required msg type

        for msgt in self.msg_types:
            if self.msg_types[msgt]['dtype'] == dbm.dtype:
                class_hash = msgt
                break
        answ = distributed_database.srv.GetDataHashDBResponse(dbm.topic_name,
                                                              class_hash,
                                                              dbm.ts, dbm.ack,
                                                              dbm.data)
        return answ

    def select_db_service_cb(self, req):
        # TODO improve for filtering
        if not len(req.robot_name):
            rospy.logerr("Error: robot_name empty")
            return
        robot_number = du.get_robot_number(self.robot_configs, req.robot_name)
        list_hashes = self.dbl.get_hash_list(robot_number)
        answ = distributed_database.srv.SelectDBResponse(list_hashes)
        return answ

    def set_ack_bit_cb(self, req):
        if not len(req.msg_hash):
            rospy.logerr("Error: msg_hash empty")
            self.shutdown()
            rospy.signal_shutdown("Error: msg_hash empty")
            rospy.spin()
        dbm = self.dbl.find_hash(req.msg_hash)
        dbm.ack = True
        checksum = self.dbl.add_modify_data(dbm=dbm)
        return distributed_database.srv.SetAckBitDBResponse(checksum)

    def shutdown(self):
        for s in self.service_list:
            s.shutdown()
