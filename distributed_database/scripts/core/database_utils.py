import struct
import rospy
import database as db
import hash_comm
import io
import pdb
import importlib

HASH_LENGTH = hash_comm.Hash.HASH_LENGTH


def get_robot_number(robot_configs, robot_name=None):
    """ Returns the current robot number, based on the name of the
    robot. If no name is provided, returns the number of the current
    robot. """

    # Check input arguments
    assert isinstance(robot_configs, dict)
    assert robot_name is not None and isinstance(robot_name, str)

    if robot_name not in robot_configs.keys():
        rospy.logerr(f"{robot_name} does not exist in robot_configs")
        rospy.signal_shutdown("robot_name does not exist in robot_configs")
        rospy.spin()
    robot_ip = robot_configs[robot_name]['IP-address']
    robot_num = int(robot_ip.split('.')[3])
    return robot_num

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

def pack_data(msg):
    """  pack data from db in the standard transmission format
    (sizes in parantheses (in bytes)):
    [
    robot (1 - unsigned char)
    topic_name_length (2 - unsigned short),
    topic_name (variable),
    data_dtype (1 unsigned char),
    priority (1 unsigned char),
    timestamp (8 - double),
    ack (1 unsigned char),
    data (variable)
    ]
    Please note that the hash is not included in the package
    """
    assert isinstance(msg, db.DBMessage)
    serial_data = struct.pack('!B', msg.robot)
    req_topic_len = len(msg.topic_name)
    serial_data += struct.pack('!H', req_topic_len)
    serial_data += msg.topic_name.encode()
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
    # unpack topic name
    p_topic_len = struct.unpack('!H',
                                  packed_data[pointer:pointer + 2])[0]
    pointer += 2
    p_topic_name = packed_data[pointer:pointer + p_topic_len].decode()
    pointer += p_topic_len
    # unpack topic type
    p_topic_dtype = struct.unpack('!B',
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

    dbm = db.DBMessage(p_robot, p_topic_name, p_topic_dtype,
                    p_priority, p_ts, p_data, p_ack)
    return dbm


def verify_checksum_msg(checksum_packed_data):
    """ Gets a checksum + packed data, splits the data and verifies the
    checksum """
    checksum = checksum_packed_data[:HASH_LENGTH].decode()
    packed_data = checksum_packed_data[HASH_LENGTH:]
    calc_checksum = hash_comm.Hash(packed_data).digest()
    return checksum == calc_checksum

def serialize_ros_msg(msg):
    # TODO check that we are not entering garbage
    sio_h = io.BytesIO()
    msg.serialize(sio_h)
    return sio_h.getvalue()


def deserialize_ros_msg(s_msg, msg_type):
    msgd = msg_type()
    msgd.deserialize(s_msg)
    return msgd

def parse_answer(api_answer, msg_types):
    constructor = msg_types[api_answer.msg_type_hash]['obj']
    msg = constructor()
    msg.deserialize(api_answer.msg_content)
    ts = api_answer.timestamp
    topic_name = api_answer.msg_name
    ack = api_answer.ack
    return topic_name, ts, msg, ack

def msg_types(topic_configs):
    """
    Extracts message types from the topic_configs dictionary and validates them. 
    Returns a dictionary with message type MD5 sums as keys and information about the message type as values.
        The value is a dictionary containing 
        - 'dtype' (an integer ID for the message type),
        - 'obj' (the message type object itself)
        - 'name' (the name of the message type).
    """
    assert isinstance(topic_configs, dict)

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
    msg_types = {}
    for i, msg in enumerate(set(msg_list)):
        package_name, msg_name = msg.split('/')
        package = importlib.import_module(package_name + '.msg')
        message_type = getattr(package, msg_name)
        msg_types[message_type._md5sum] = {"dtype": i,
                                                "obj": message_type,
                                                "name": msg}
    return msg_types
