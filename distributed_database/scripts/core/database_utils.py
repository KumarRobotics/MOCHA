import struct
import rospy
import database as db
import hash_comm
import io
import pdb
import importlib
import random

HEADER_LENGTH = hash_comm.TsHeader.HEADER_LENGTH


def get_robot_number(robot_configs, robot_name):
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


def get_topic_id_from_name(robot_configs, topic_configs, robot_name, topic_name):
    """ Returns the topic id for a particular robot,
    by searching the topic name"""
    assert isinstance(robot_configs, dict)
    assert isinstance(topic_configs, dict)
    assert robot_name is not None and isinstance(robot_name, str)
    assert topic_name is not None and isinstance(topic_name, str)

    if robot_name not in robot_configs.keys():
        rospy.logerr(f"{robot_name} does not exist in robot_configs")
        rospy.signal_shutdown("robot_name does not exist in robot_configs")
        rospy.spin()

    list_topics = topic_configs[robot_configs[robot_name]["node-type"]]
    id = None
    for i, topic in enumerate(list_topics):
        if topic_name == topic["msg_topic"]:
            id = i
            break
    if id is None:
        rospy.logerr(f"{topic_name} does not exist in topic_configs")
        rospy.signal_shutdown("topic_name does not exist in topic_configs")
        rospy.spin()
    return id


def get_topic_name_from_id(robot_configs, topic_configs, robot_name, topic_id):
    """ Returns the topic name for a particular robot, given its id"""
    assert isinstance(robot_configs, dict)
    assert isinstance(topic_configs, dict)
    assert robot_name is not None and isinstance(robot_name, str)
    assert topic_id is not None and isinstance(topic_id, int)
    list_topics = topic_configs[robot_configs[robot_name]["node-type"]]
    if topic_id >= len(list_topics):
        rospy.logerr(f"{topic_id} does not exist in topic_configs")
        rospy.signal_shutdown("topic_id does not exist in topic_configs")
        rospy.spin()
    return list_topics[topic_id]["msg_topic"]


def serialize_headers(header_list):
    """ Converts a list of headers (bytes) into a single stream """
    return b''.join(header_list)


def deserialize_headers(serial_headers):
    """ Converts a stream of hashes into a list of hashes """
    if len(serial_headers) % HEADER_LENGTH != 0:
        raise Exception('deserialize_hashes: wrong length of string')
    splitted_headers = [serial_headers[i:i+HEADER_LENGTH]
                        for i in
                        range(0, len(serial_headers), HEADER_LENGTH)]
    return splitted_headers

def pack_data(msg):
    """  pack data from db in the standard transmission format
    (sizes in parantheses (in bytes)):
    [
    data_dtype (1 unsigned char),
    priority (1 unsigned char),
    data -- all the data
    ]

    Note that all the components that are part of the header are not packed in
    the data
    """
    assert isinstance(msg, db.DBMessage)
    serial_data = struct.pack('!B', msg.dtype)
    serial_data += struct.pack('!B', msg.priority)
    serial_data += msg.data
    return serial_data


def unpack_data(header, packed_data):
    """ Unpacks a package with data, and return a dict to insert in the
    database. We _could_ do the unpack in one operation, but this is
    easier to debug. """

    h = hash_comm.TsHeader.from_header(header)
    p_robot_id, p_topic_id, p_ts = h.get_id_and_time()

    pointer = 0
    # unpack topic type
    p_topic_dtype = struct.unpack('!B',
                                    packed_data[pointer:pointer + 1])[0]
    pointer += 1
    # unpack priority
    p_priority = struct.unpack('!B',
                               packed_data[pointer:pointer + 1])[0]
    pointer += 1
    # unpack data
    p_data = packed_data[pointer:]
    assert len(p_data) != 0, "Empty data in unpack_data"

    dbm = db.DBMessage(p_robot_id, p_topic_id, p_topic_dtype,
                       p_priority, p_ts, p_data)
    return dbm


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
                rospy.signal_shutdown("Error: msg_type {msg} not valid")
                rospy.spin()
            msg_list.append(topic['msg_type'])
    # Important: sort the msg_list so we have a deterministic order
    msg_list.sort()

    msg_types = {}
    for i, msg in enumerate(msg_list):
        package_name, msg_name = msg.split('/')
        package = importlib.import_module(package_name + '.msg')
        message_type = getattr(package, msg_name)
        msg_types[message_type._md5sum] = {"dtype": i,
                                           "obj": message_type,
                                           "name": msg}
    return msg_types

def generate_random_header():
    # generate random robot_id and topic_id, between 0 and 255
    robot_id = random.randint(0, 255)
    topic_id = random.randint(0, 255)
    # Generate a random rospy timestamp
    time = rospy.Time.from_sec(random.random())
    h = hash_comm.TsHeader.from_data(robot_id, topic_id, time)
    return h.bindigest()
