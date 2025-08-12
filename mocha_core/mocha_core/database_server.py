#!/usr/bin/env python3

import os
import threading
import rospy
import mocha_core.srv
import database
import pdb
import database_utils as du


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

        if self.robot_name not in self.robot_configs.keys():
            rospy.logerr(f"{self.robot_name} does not exist in robot_configs")
            rospy.signal_shutdown("robot_name does not exist in robot_configs")
            rospy.spin()

        self.topic_list = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]

        # Create the empty database with lock
        self.dbl = database.DBwLock()

        # Get current robot number
        self.robot_number = du.get_robot_id_from_name(self.robot_configs,
                                                      robot_name=self.robot_name)

        # create services for all the possible calls to the DB
        self.service_list = []
        s = rospy.Service('~AddUpdateDB',
                          mocha_core.srv.AddUpdateDB,
                          self.add_update_db_service_cb)
        self.service_list.append(s)
        s = rospy.Service('~GetDataHeaderDB',
                          mocha_core.srv.GetDataHeaderDB,
                          self.get_data_hash_db_service_cb)
        self.service_list.append(s)
        s = rospy.Service('~SelectDB',
                          mocha_core.srv.SelectDB,
                          self.select_db_service_cb)
        self.service_list.append(s)

        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs)

    def add_update_db_service_cb(self, req):
        if not isinstance(req.topic_id, int) or req.topic_id is None:
            rospy.logerr("Error: topic_id empty")
            return
        if len(req.msg_content) == 0:
            rospy.logerr("Error: msg_content empty")
            return
        if req.topic_id > len(self.topic_list):
            rospy.logerr("Error: topic_id not in topic_list")
            return
        topic = self.topic_list[req.topic_id]
        priority = du.get_priority_number(topic["msg_priority"])
        ts = req.timestamp
        ts = rospy.Time(ts.secs, ts.nsecs)
        dbm = database.DBMessage(self.robot_number,
                                 req.topic_id,
                                 dtype=self.msg_types[self.robot_number][req.topic_id]["dtype"],
                                 priority=priority,
                                 ts=ts,
                                 data=req.msg_content)

        header = self.dbl.add_modify_data(dbm)
        return mocha_core.srv.AddUpdateDBResponse(header)

    def get_data_hash_db_service_cb(self, req):
        if req.msg_header is None or len(req.msg_header) == 0:
            rospy.logerr("Error: msg_header empty")
            return
        dbm = self.dbl.find_header(req.msg_header)
        answ = mocha_core.srv.GetDataHeaderDBResponse(dbm.robot_id,
                                                                dbm.topic_id,
                                                                dbm.ts,
                                                                dbm.data)
        return answ

    def select_db_service_cb(self, req):
        # TODO Implement filtering
        if req.robot_id is None:
            rospy.logerr("Error: robot_id none")
            return
        if req.topic_id is None:
            rospy.logerr("Error: topic_id none")
            return
        list_headers = self.dbl.get_header_list(req.robot_id)
        answ = mocha_core.srv.SelectDBResponse(du.serialize_headers(list_headers))
        return answ

    def shutdown(self):
        for s in self.service_list:
            s.shutdown()
