#!/usr/bin/env python3
import threading
import hash_comm
import rospy
import pdb
import database_utils as du
import numpy as np
import hash_comm
import copy


class DBMessage():
    """Database message object.

    This class is used to represent messages in the context of a database.
    The messages are typically used for inserting or extracting data from
    the database.
    """
    def __init__(self, robot_id, topic_id,
                 dtype, priority, ts, data):
        """
        Initialize the DBMessage object.

        Args:
            robot_id (int): ID of the robot.
            topic_id (int): Topic id of the robot
            dtype (int): Data type identifier.
            priority (int): Priority level of the message.
            ts (rostime): Timestamp of the message
            data (bytes): Binary data payload.
        """
        assert isinstance(robot_id, int)
        assert isinstance(topic_id, int)
        assert isinstance(dtype, int)
        assert isinstance(priority, int)
        assert isinstance(ts, rospy.Time)
        assert isinstance(data, bytes)
        # The first three items are encoded in the header
        self.robot_id = robot_id
        self.topic_id = topic_id
        self.ts = ts
        # The next items are encoded in the data
        self.dtype = dtype
        self.priority = priority
        self.data = data
        # Calculate the header of the message
        header = hash_comm.TsHeader.from_data(self.robot_id,
                                              self.topic_id, self.ts)
        self.header = header.bindigest()

    def __eq__(self, other):
        if not isinstance(other, DBMessage):
            return False
        if other.robot_id != self.robot_id:
            return False
        if other.topic_id != self.topic_id:
            return False
        if other.dtype != self.dtype:
            return False
        if other.priority != self.priority:
            return False
        if other.ts.secs != self.ts.secs:
            return False
        if np.abs(other.ts.nsecs - self.ts.nsecs) > 1000000:
            print("nsecs diff: %d" % np.abs(other.ts.nsecs - self.ts.nsecs))
            return False
        if other.data != self.data:
            return False
        return True

    def __str__(self):
        return "%d, %d, %d, %d, %f" % (self.robot_id, self.topic_id,
                                       self.dtype, self.priority,
                                       self.ts)


class DBwLock():
    """ Database with lock object

    The database is a dictionary.
    The lock should be manually acquired before executing any operation
    with the database to ensure data integrity.

    sample_db may be a valid dictionary that can be preloaded into
    the object (useful for debugging, see sample_db.py) """
    def __init__(self, sample_db=None):
        if sample_db is not None:
            assert isinstance(sample_db, dict)
            self.db = copy.deepcopy(sample_db)
        else:
            self.db = {}
        self.lock = threading.Lock()

    def add_modify_data(self, dbm):
        """ Insert new stuff into the db. Use the header as message index """
        # Do a quick check
        assert isinstance(dbm, DBMessage)

        # Acquire lock and commit into db
        self.lock.acquire()
        # Create and store things in db
        if dbm.robot_id not in self.db:
            self.db[dbm.robot_id] = {}
        if dbm.topic_id not in self.db[dbm.robot_id]:
            self.db[dbm.robot_id][dbm.topic_id] = {}
        self.db[dbm.robot_id][dbm.topic_id][dbm.header] = dbm
        self.lock.release()
        return dbm.header

    def get_header_list(self, filter_robot_id=None, filter_latest=None):
        """ Returns a list with all the headers of the db. The results are
        filtered for a specific robot, or *after* a specific timestamp if
        these fields are available.  To filter by timestamp,
        you should specify a robot. Timestamps are recorded in the robot
        frame (i.e. each robot has different timestamps, so it does not make
        sense to filter by a global timestamp) """
        if filter_robot_id is not None:
            assert isinstance(filter_robot_id, int)
        if filter_latest is not None:
            assert isinstance(filter_latest, bool)

        # Header list is a dict with the headers as keys and the priorities as
        # values
        header_list = {}

        # To avoid inconsistencies, the db is locked while searching
        self.lock.acquire()
        for robot_id in self.db:
            if filter_robot_id is not None and robot_id != filter_robot_id:
                continue
            for topic in self.db[robot_id]:
                if filter_latest:
                    latest_msg_ts = rospy.Time(1, 0)
                    latest_msg = None
                for header in self.db[robot_id][topic]:
                    msg_content = self.db[robot_id][topic][header]
                    if filter_latest and msg_content.ts > latest_msg_ts:
                        latest_msg_ts = msg_content.ts
                        latest_msg = msg_content
                    if not filter_latest:
                        header_list[header] = {'prio': msg_content.priority,
                                               'ts': msg_content.ts}
                if filter_latest:
                    header_list[latest_msg.header] = {'prio': latest_msg.priority,
                                                      'ts': latest_msg.ts}
        self.lock.release()

        # Sort the dictionary by value, and get the keys
        sorted_tuples = sorted(header_list.items(),
                               key=lambda kv: (kv[1]['prio'], kv[1]['ts'], kv[0]),
                               reverse=True)
        sorted_header_list = [i[0] for i in sorted_tuples]
        return sorted_header_list

    def get_ts_dict(self):
        """ Returns a dictionary with the newest timestamps of all the topics """
        ts_dict = {}
        self.lock.acquire()
        for robot_id in self.db:
            if robot_id not in ts_dict:
                ts_dict[robot_id] = {}
            for topic in self.db[robot_id]:
                if topic not in ts_dict[robot_id]:
                    ts_dict[robot_id][topic] = -np.inf
                for header in self.db[robot_id][topic]:
                    msg = self.db[robot_id][topic][header]
                    ts_dict[robot_id][topic] = max(ts_dict[robot_id][topic],
                                                   msg.ts.to_sec())
        self.lock.release()
        return ts_dict

    def headers_not_in_local(self, remote_header_list, newer=False):
        # Compares a remote header list with the local database.
        # If newer is True, only headers newer than the local ones are
        # returned. Otherwise, all the headers are returned.
        assert isinstance(remote_header_list, list)
        assert isinstance(newer, bool)

        missing_headers = []
        if not newer:
            local_headers = self.get_header_list()
            for h in remote_header_list:
                if h not in local_headers:
                    missing_headers.append(h)
            return missing_headers
        else:
            ts_dict = self.get_ts_dict()
            for h in remote_header_list:
                h = hash_comm.TsHeader.from_header(h)
                r_id, t_id, time = h.get_id_and_time()
                if r_id not in ts_dict:
                    missing_headers.append(h.bindigest())
                elif t_id not in ts_dict[h.robot_id]:
                    missing_headers.append(h.bindigest())
                elif time.to_sec() > ts_dict[h.robot_id][h.topic_id]:
                    missing_headers.append(h.bindigest())
            return missing_headers

    def find_header(self, requested_header):
        assert isinstance(requested_header, bytes)
        # Find data in db
        data_found = False
        self.lock.acquire()
        for robot in self.db:
            for topic_id in self.db[robot]:
                if data_found:
                    break
                for header in self.db[robot][topic_id]:
                    if data_found:
                        break
                    if header == requested_header:
                        data_found = True
                        req_robot_id = robot
                        req_topic_id = topic_id
                        break
        if not data_found:
            self.lock.release()
            raise Exception('packData: header not found')
        req_dtype = self.db[req_robot_id][req_topic_id][header].dtype
        req_priority = self.db[req_robot_id][req_topic_id][header].priority
        req_ts = self.db[req_robot_id][req_topic_id][header].ts
        req_data = self.db[req_robot_id][req_topic_id][header].data
        self.lock.release()
        dbm = DBMessage(req_robot_id, req_topic_id, req_dtype,
                        req_priority, req_ts, req_data)
        return dbm

    def __str__(self):
        # Print the database in a nice way
        self.lock.acquire()
        out = ""
        for robot in self.db:
            out += f"Robot ID: {robot}\n"
            for topic_id in self.db[robot]:
                out += f"  Topic ID: {topic_id}\n"
                for header in self.db[robot][topic_id]:
                    msg = self.db[robot][topic_id][header]
                    out += f"    TS: {str(msg.ts)}\n"
                    out += f"      Dtype: {msg.dtype}\n"
                    out += f"      Prio: {msg.priority}\n"
                    out += f"      Data: {msg.data}\n"
                    out += f"      Header: {header}\n"
        self.lock.release()
        return out
