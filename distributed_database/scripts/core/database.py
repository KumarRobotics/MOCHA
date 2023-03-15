#!/usr/bin/env python3
import threading
import hash_comm
import pdb
import database_utils as du


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
            self.db = sample_db
        else:
            self.db = {}
        self.lock = threading.Lock()

    def add_modify_data(self, dbm):
        """ Insert new stuff into the db. When called without data, only the
        hearbeat gets updated """
        # Do a quick check
        assert isinstance(dbm, DBMessage)

        # Acquire lock and commit into db
        self.lock.acquire()
        # Create and store things in db
        if dbm.robot not in self.db:
            self.db[dbm.robot] = {}
        if dbm.topic_name not in self.db[dbm.robot]:
            self.db[dbm.robot][dbm.topic_name] = {}
        self.db[dbm.robot][dbm.topic_name]['dtype'] = dbm.dtype
        self.db[dbm.robot][dbm.topic_name]['priority'] = dbm.priority
        self.db[dbm.robot][dbm.topic_name]['ts'] = dbm.ts
        self.db[dbm.robot][dbm.topic_name]['data'] = dbm.data
        self.db[dbm.robot][dbm.topic_name]['ack'] = dbm.ack
        packed_data = du.pack_data(dbm)
        checksum_data = hash_comm.Hash(packed_data).digest()
        self.db[dbm.robot][dbm.topic_name]['hash'] = checksum_data
        self.lock.release()
        return checksum_data

    def get_hash_list(self, filter_robot=None, filter_ts=None):
        """ Returns a list with all the hashes of the db. The results are
        filtered for a specific robot, or *after* a specific timestamp if
        these fields are available.  To filter by timestamp,
        you should specify a robot. Timestamps are recorded in the robot
        frame (i.e. each robot has different timestamps, so it does not make
        sense to filter by a global timestamp) """

        # Hash list is a dict with the hashes as keys and the priorities as
        # values
        hash_list = {}
        if filter_robot is not None:
            assert isinstance(filter_robot, int)
        if filter_ts is not None:
            assert isinstance(filter_ts, (float, int))
            if filter_robot is None:
                raise Exception("get_hash_list_from_db: ts without robot")

        # To avoid inconsistencies, the db is locked while searching
        self.lock.acquire()
        for robot in self.db:
            if filter_robot is not None and robot != filter_robot:
                continue
            for tag in self.db[robot]:
                if filter_ts is not None and self.db[robot][tag]['ts'] <= filter_ts:
                    continue
                hash_list[self.db[robot][tag]['hash']] = {'prio': self.db[robot][tag]['priority'],
                        'ts': self.db[robot][tag]['ts']}
        self.lock.release()

        # Sort the dictionary by value, and get the keys
        sorted_tuples = sorted(hash_list.items(),
                               key=lambda kv: (kv[1]['prio'], kv[1]['ts'], kv[0]),
                               reverse=False)
        sorted_hash_list = [i[0] for i in sorted_tuples]
        return sorted_hash_list

    def hashes_not_in_local(self, remote_hash_list):
        local_hashes = self.get_hash_list()
        missing_hashes = []
        for h in remote_hash_list:
            if h not in local_hashes:
                missing_hashes.append(h)
        return missing_hashes


    def find_hash(self, requested_hash):
        # Find data in db
        data_found = False
        self.lock.acquire()
        for robot in self.db:
            for topic_name in self.db[robot]:
                if self.db[robot][topic_name]['hash'] == requested_hash:
                    data_found = True
                    req_robot = robot
                    req_topic_name = topic_name
        if not data_found:
            self.lock.release()
            raise Exception('packData: hash not found')
        req_dtype = self.db[req_robot][req_topic_name]['dtype']
        req_priority = self.db[req_robot][req_topic_name]['priority']
        req_ts = self.db[req_robot][req_topic_name]['ts']
        req_data = self.db[req_robot][req_topic_name]['data']
        req_ack = self.db[req_robot][req_topic_name]['ack']
        self.lock.release()
        dbm = DBMessage(req_robot, req_topic_name, req_dtype,
                        req_priority, req_ts, req_data, req_ack)
        return dbm


class DBMessage():
    """Database message object.

    This class is used to represent messages in the context of a database.
    The messages are typically used for inserting or extracting data from
    the database.
    """
    def __init__(self, robot, topic_name,
                 dtype, priority, ts, data, ack):
        """
        Initialize the DBMessage object.

        Args:
            robot (int): ID of the robot.
            topic_name (str): The name of the topic_name.
            dtype (int): Data type identifier.
            priority (int): Priority level of the message.
            ts (float): Timestamp of the message.
            data (bytes): Binary data payload.
            ack (bool): Acknowledgement status.
        """
        self.robot = robot
        self.topic_name = topic_name
        self.dtype = dtype
        self.priority = priority
        self.ts = ts
        self.data = data
        self.ack = ack
        self.check_msg()

    def check_msg(self):
        assert isinstance(self.robot, int)
        assert isinstance(self.topic_name, str)
        assert isinstance(self.dtype, int)
        assert isinstance(self.priority, int)
        assert isinstance(self.ts, float)
        assert isinstance(self.ack, bool)
        assert isinstance(self.data, bytes)

    def __eq__(self, other):
        if not isinstance(other, DBMessage):
            return False
        if other.robot != self.robot:
            return False
        if other.topic_name != self.topic_name:
            return False
        if other.dtype != self.dtype:
            return False
        if other.priority != self.priority:
            return False
        if abs(other.ts - self.ts) > 1e-8:
            return False
        if other.data != self.data:
            return False
        if other.ack != self.ack:
            return False
        return True

    def __str__(self):
        return "%d, %s, %d, %d, %f, %d" % (self.robot, self.topic_name,
                                           self.dtype, self.priority,
                                           self.ts, self.ack)
