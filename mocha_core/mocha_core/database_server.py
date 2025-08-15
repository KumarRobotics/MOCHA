#!/usr/bin/env python3

import os
import threading
import rclpy
import rclpy.logging
import rclpy.time
import mocha_core.srv
import mocha_core.database as database
import pdb
import mocha_core.database_utils as du


class DatabaseServer:
    """ Starts a clean database and offers an API-like service
    to interact with the databaseself.

    Please see the list of services in the srv folder
    """
    def __init__(self, robot_configs, topic_configs, node=None):
        # Check input topics
        assert isinstance(robot_configs, dict)
        assert isinstance(topic_configs, dict)

        self.robot_configs = robot_configs
        self.topic_configs = topic_configs
        
        # Store or create the ROS2 node
        self.node = node
        if self.node is None:
            self.logger = rclpy.logging.get_logger('database_server')
        else:
            self.logger = self.node.get_logger()

        # Get current robot name from params (passed as parameter since ROS2 doesn't use global params)
        # For now, we'll default to 'charon' - this should be passed as a parameter
        self.robot_name = 'charon'  # TODO: Make this configurable

        if self.robot_name not in self.robot_configs.keys():
            self.logger.error(f"{self.robot_name} does not exist in robot_configs")
            raise ValueError("robot_name does not exist in robot_configs")

        self.topic_list = self.topic_configs[self.robot_configs[self.robot_name]["node-type"]]

        # Create the empty database with lock
        self.dbl = database.DBwLock()

        # Get current robot number
        self.robot_number = du.get_robot_id_from_name(self.robot_configs,
                                                      robot_name=self.robot_name)

        # create services for all the possible calls to the DB
        self.service_list = []
        if self.node is not None:
            s = self.node.create_service(mocha_core.srv.AddUpdateDB,
                                        'AddUpdateDB',
                                        self.add_update_db_service_cb)
            self.service_list.append(s)
            s = self.node.create_service(mocha_core.srv.GetDataHeaderDB,
                                        'GetDataHeaderDB',
                                        self.get_data_hash_db_service_cb)
            self.service_list.append(s)
            s = self.node.create_service(mocha_core.srv.SelectDB,
                                        'SelectDB',
                                        self.select_db_service_cb)
            self.service_list.append(s)

        self.msg_types = du.msg_types(self.robot_configs, self.topic_configs)

    def add_update_db_service_cb(self, req):
        try:
            if not isinstance(req.topic_id, int) or req.topic_id is None:
                self.logger.error("topic_id empty")
                return mocha_core.srv.AddUpdateDB.Response()
            if len(req.msg_content) == 0:
                self.logger.error("msg_content empty")
                return mocha_core.srv.AddUpdateDB.Response()
            if req.topic_id >= len(self.topic_list):  # Changed > to >= for proper bounds check
                self.logger.error(f"topic_id {req.topic_id} not in topic_list (length: {len(self.topic_list)})")
                return mocha_core.srv.AddUpdateDB.Response()
            
            topic = self.topic_list[req.topic_id]
            priority = du.get_priority_number(topic["msg_priority"])
            ts = req.timestamp
            # ROS2 builtin_interfaces/Time uses 'sec' and 'nanosec' fields
            ts = rclpy.time.Time(seconds=ts.sec, nanoseconds=ts.nanosec)
            
            # Convert array to bytes if needed (ROS2 service messages use array)
            msg_data = req.msg_content
            if hasattr(msg_data, 'tobytes'):
                msg_data = msg_data.tobytes()
            elif isinstance(msg_data, (list, tuple)):
                msg_data = bytes(msg_data)
            
            dbm = database.DBMessage(self.robot_number,
                                     req.topic_id,
                                     dtype=self.msg_types[self.robot_number][req.topic_id]["dtype"],
                                     priority=priority,
                                     ts=ts,
                                     data=msg_data)

            header = self.dbl.add_modify_data(dbm)
            response = mocha_core.srv.AddUpdateDB.Response()
            response.new_header = header
            return response
        except Exception as e:
            self.logger.error(f"Exception in add_update_db_service_cb: {e}")
            return mocha_core.srv.AddUpdateDB.Response()

    def get_data_hash_db_service_cb(self, req):
        try:
            if req.msg_header is None or len(req.msg_header) == 0:
                self.logger.error("msg_header empty")
                return mocha_core.srv.GetDataHeaderDB.Response()
            
            # Convert array to bytes if needed (ROS2 service messages use array)
            header_data = req.msg_header
            if hasattr(header_data, 'tobytes'):
                header_data = header_data.tobytes()
            elif isinstance(header_data, (list, tuple)):
                header_data = bytes(header_data)
            
            dbm = self.dbl.find_header(header_data)
            
            response = mocha_core.srv.GetDataHeaderDB.Response()
            response.robot_id = dbm.robot_id
            response.topic_id = dbm.topic_id
            response.timestamp = dbm.ts
            response.msg_content = dbm.data
            return response
        except Exception as e:
            self.logger.error(f"Exception in get_data_hash_db_service_cb: {e}")
            return mocha_core.srv.GetDataHeaderDB.Response()

    def select_db_service_cb(self, req):
        try:
            # TODO Implement filtering
            response = mocha_core.srv.SelectDB.Response()
            
            # Note: robot_id and topic_id are uint8 in ROS2, so they can't be None
            # We can add range validation if needed, but for now just proceed
                
            list_headers = self.dbl.get_header_list(req.robot_id)
            
            response.headers = du.serialize_headers(list_headers)
            return response
        except Exception as e:
            self.logger.error(f"Exception in select_db_service_cb: {e}")
            response = mocha_core.srv.SelectDB.Response()
            response.headers = b''
            return response

    def shutdown(self):
        # In ROS2, services are automatically cleaned up when the node is destroyed
        pass
