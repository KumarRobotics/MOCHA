#!/usr/bin/env python3

import os
import threading
import rospy
import distributed_database.srv
import pdb
import synchronize_utils as su
import database_server_utils as du

NODE_NAME = "database_server"


class DatabaseServer:
    """ Starts a clean database and offers an API-like service
    to interact with the database. """
    def __init__(self, config_file, robot_name=None):
        # Store config file
        self.config_file = config_file
        # Create the empty database with lock
        self.dbl = su.DBwLock()
        # Get current robot number
        self.robot = du.get_robot_number(config_file, robot_name=robot_name)
        # create services for all the possible calls to the DB
        rospy.Service(os.path.join(NODE_NAME, 'AddUpdateDB'),
                      distributed_database.srv.AddUpdateDB,
                      self.add_update_db_service_cb)
        rospy.Service(os.path.join(NODE_NAME, 'GetDataHashDB'),
                      distributed_database.srv.GetDataHashDB,
                      self.get_data_hash_db_service_cb)
        rospy.Service(os.path.join(NODE_NAME, 'SelectDB'),
                      distributed_database.srv.SelectDB,
                      self.select_db_service_cb)
        rospy.Service(os.path.join(NODE_NAME, 'SetAckBitDB'),
                      distributed_database.srv.SetAckBitDB,
                      self.set_ack_bit_cb)

    def add_update_db_service_cb(self, req):
        curr_time = rospy.get_time()
        if not len(req.msg_name):
            rospy.logerr("Error: msg_name empty")
            return
        dbm = su.DBMessage(self.robot,
                           req.msg_name,
                           du.MSG_TYPES[req.msg_type_hash]['dtype'],
                           du.MSG_TYPES[req.msg_type_hash]['priority'],
                           curr_time,
                           req.msg_content, False)

        checksum = du.add_modify_data_dbl(dbl=self.dbl,
                                          dbm=dbm)
        return distributed_database.srv.AddUpdateDBResponse(checksum)

    def get_data_hash_db_service_cb(self, req):
        if not len(req.msg_hash):
            rospy.logerr("Error: msg_hash empty")
            return
        dbm = su.find_hash_dbl(self.dbl, req.msg_hash)
        # Find key for required msg type
        for msgt in du.MSG_TYPES:
            if du.MSG_TYPES[msgt]['dtype'] == dbm.dtype:
                class_hash = msgt
                break
        answ = distributed_database.srv.GetDataHashDBResponse(dbm.feature_name,
                class_hash, dbm.ts, dbm.ack, dbm.data)
        return answ

    def select_db_service_cb(self, req):
        # TODO improve for filtering
        if not len(req.robot_name):
            rospy.logerr("Error: robot_name empty")
            return
        robot_number = du.get_robot_number(self.config_file, req.robot_name)
        list_hashes = su.get_hash_list_from_dbl(self.dbl, robot_number)
        answ = distributed_database.srv.SelectDBResponse(list_hashes)
        return answ

    def set_ack_bit_cb(self, req):
        if not len(req.msg_hash):
            rospy.logerr("Error: msg_hash empty")
            return
        dbm = su.find_hash_dbl(self.dbl, req.msg_hash)
        dbm.ack = True
        checksum = du.add_modify_data_dbl(dbl=self.dbl,
                                          dbm=dbm)
        return distributed_database.srv.SetAckBitDBResponse(checksum)


if __name__ == "__main__":
    rospy.init_node(os.path.join(NODE_NAME))
    # For standalone execution, run with test configs
    CONFIG_FILE = "testConfigs/robotConfigs.yml"
    ROBOT_NAME = "charon"
    DatabaseServer(CONFIG_FILE, ROBOT_NAME)
    rospy.spin()
