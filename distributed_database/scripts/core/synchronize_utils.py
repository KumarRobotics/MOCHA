#!/usr/bin/env python3
import threading
import struct
import hash_comm

HASH_LENGTH = hash_comm.Hash.HASH_LENGTH

class DBwLock():
    """ Database with lock object

    The lock should be manually acquired before executing any operation
    with the database to ensure data integrity. sample_db may be a valid
    dictionary that can be preloaded into the object (useful for
    debugging) """
    def __init__(self, sample_db=None):
        if sample_db is not None:
            assert isinstance(sample_db, dict)
            self.db = sample_db
        else:
            self.db = {}
        self.lock = threading.Lock()


class DBMessage():
    """ Database message obsect

    The databases that are inserted/extracted from the DB use the
    following object """
    def __init__(self, robot, feature_name,
                 dtype, priority, ts, data, ack):
        self.robot = robot
        self.feature_name = feature_name
        self.dtype = dtype
        self.priority = priority
        self.ts = ts
        self.data = data
        self.ack = ack
        self.check_msg()

    def check_msg(self):
        assert isinstance(self.robot, int)
        assert isinstance(self.feature_name, str)
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
        if other.feature_name != self.feature_name:
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
        return "%d, %s, %d, %d, %f, %d" % (self.robot, self.feature_name,
                                           self.dtype, self.priority,
                                           self.ts, self.ack)


def get_hash_list_from_dbl(dbl, filter_robot=None, filter_ts=None):
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
    dbl.lock.acquire()
    for robot in dbl.db:
        if filter_robot is not None and robot != filter_robot:
            continue
        for tag in dbl.db[robot]:
            if filter_ts is not None and dbl.db[robot][tag]['ts'] <= filter_ts:
                continue
            hash_list[dbl.db[robot][tag]['hash']] = dbl.db[robot][tag]['priority']
    dbl.lock.release()

    # Sort the dictionary by value, and get the keys
    sorted_tuples = sorted(hash_list.items(),
                           key=lambda kv: (kv[1], kv[0]),
                           reverse=True)
    sorted_hash_list = [i[0] for i in sorted_tuples]
    return sorted_hash_list


def serialize_hashes(hash_list):
    """ Converts a list of hashes into a single stream """
    return ''.join(hash_list).encode()


def deserialize_hashes(serial_hashes):
    """ Converts a stream of hashes into a list of hashes """
    decoded_hashes = serial_hashes.decode()
    if len(decoded_hashes) % HASH_LENGTH != 0:
        raise Exception('deserialize_hashes: wrong length of string')
    splitted_hashes = [decoded_hashes[i:i+HASH_LENGTH]
                       for i in
                       range(0, len(decoded_hashes), HASH_LENGTH)]
    return splitted_hashes


def hashes_not_in_local(dbl, remote_hash_list):
    local_hashes = get_hash_list_from_dbl(dbl)
    missing_hashes = []
    for h in remote_hash_list:
        if h not in local_hashes:
            missing_hashes.append(h)
    return missing_hashes


def find_hash_dbl(dbl, requested_hash):
    # Find data in db
    data_found = False
    dbl.lock.acquire()
    for robot in dbl.db:
        for feature in dbl.db[robot]:
            if dbl.db[robot][feature]['hash'] == requested_hash:
                data_found = True
                req_robot = robot
                req_feature_name = feature
    if not data_found:
        dbl.lock.release()
        raise Exception('packData: hash not found')
    req_dtype = dbl.db[req_robot][req_feature_name]['dtype']
    req_priority = dbl.db[req_robot][req_feature_name]['priority']
    req_ts = dbl.db[req_robot][req_feature_name]['ts']
    req_data = dbl.db[req_robot][req_feature_name]['data']
    req_ack = dbl.db[req_robot][req_feature_name]['ack']
    dbl.lock.release()
    dbm = DBMessage(req_robot, req_feature_name, req_dtype,
                    req_priority, req_ts, req_data, req_ack)
    return dbm


def pack_data(msg):
    """  pack data from db in the standard transmission format
    (sizes in parantheses (in bytes)):
    [
    robot (1 - unsigned char)
    feature_name_length (2 - unsigned short),
    feature_name (variable),
    feature_dtype (1 unsigned char),
    priority (1 unsigned char),
    timestamp (8 - double),
    ack (1 unsigned char),
    data (variable)
    ]
    Please note that the hash is not included in the package
    """
    assert isinstance(msg, DBMessage)
    serial_data = struct.pack('!B', msg.robot)
    req_feature_len = len(msg.feature_name)
    serial_data += struct.pack('!H', req_feature_len)
    serial_data += msg.feature_name.encode()
    serial_data += struct.pack('!B', msg.dtype)
    serial_data += struct.pack('!B', msg.priority)
    serial_data += struct.pack('!d', msg.ts)
    serial_data += struct.pack('!B', int(msg.ack))
    serial_data += msg.data
    return serial_data


def unpack_data(packed_data):
    """ Unpacks a package with data, and return a dict to insert in the
    database. We _could_ do the unpack in one operation, but this is
    easier to debug. """

    # unpack robot
    p_robot = struct.unpack('!B', packed_data[:1])[0]
    pointer = 1
    # unpack feature name
    p_feature_len = struct.unpack('!H',
                                  packed_data[pointer:pointer + 2])[0]
    pointer += 2
    p_feature_name = packed_data[pointer:pointer + p_feature_len].decode()
    pointer += p_feature_len
    # unpack feature type
    p_feature_dtype = struct.unpack('!B',
                                    packed_data[pointer:pointer + 1])[0]
    pointer += 1
    # unpack priority
    p_priority = struct.unpack('!B',
                               packed_data[pointer:pointer + 1])[0]
    pointer += 1
    # unpack timestamp
    p_ts = struct.unpack('!d', packed_data[pointer:pointer + 8])[0]
    pointer += 8
    # unpack ack
    p_ack = struct.unpack('!B',
                          packed_data[pointer:pointer + 1])[0]
    p_ack = bool(p_ack)
    pointer += 1
    # unpack data
    p_data = packed_data[pointer:]
    assert len(p_data) != 0, "Empty data in unpack_data"

    dbm = DBMessage(p_robot, p_feature_name, p_feature_dtype,
                    p_priority, p_ts, p_data, p_ack)
    return dbm


def verify_checksum_msg(checksum_packed_data):
    """ Gets a checksum + packed data, splits the data and verifies the
    checksum """
    checksum = checksum_packed_data[:HASH_LENGTH].decode()
    packed_data = checksum_packed_data[HASH_LENGTH:]
    calc_checksum = hash_comm.Hash(packed_data).digest()
    return checksum == calc_checksum
