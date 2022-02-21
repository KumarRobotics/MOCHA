import os
import yaml
import rospkg
import rospy
import synchronize_utils as su
import distributed_database.msg
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import hash_comm
import io

MSG_TYPES = {
    nav_msgs.msg.Odometry._md5sum: {'dtype': 1,
        'priority': 3, 'obj': nav_msgs.msg.Odometry},
    sensor_msgs.msg.PointCloud2._md5sum: {'dtype': 2,
        'priority': 3, 'obj': sensor_msgs.msg.PointCloud2},
    geometry_msgs.msg.PointStamped._md5sum: {'dtype': 3,
        'priority': 3, 'obj': geometry_msgs.msg.PointStamped},
    sensor_msgs.msg.Image._md5sum: {'dtype': 4,
        'priority': 3, 'obj': sensor_msgs.msg.Image},
    geometry_msgs.msg.PoseStamped._md5sum: {'dtype': 5,
        'priority': 3, 'obj': geometry_msgs.msg.PoseStamped},
    nav_msgs.msg.Path._md5sum: {'dtype': 6,
        'priority': 3, 'obj': nav_msgs.msg.Path},
}


def get_robot_number(config_file, robot_name=None):
    """ Returns the current robot number, based on the name of the
    robot. If no name is provided, returns the number of the current
    robot. """
    if robot_name is None:
        robot_ip = rospy.get_param(os.path.join('integrate_database',
                                                'client_ip'))
    else:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('network_configs')
        robot_yaml_path = os.path.join(package_path, "config", config_file)
        with open(robot_yaml_path, 'r') as f:
            robot_cfg = yaml.load(f)
        robot_list = robot_cfg.keys()
        if robot_name not in robot_list:
            raise Exception("robot_name does not exist in " + config_file)
        robot_ip = robot_cfg[robot_name]['IP-address']
    robot_num = int(robot_ip.split('.')[3])
    return robot_num


def get_robot_name(config_file):
    """ Returns the robot name, fetched from the current ip """
    robot_num = get_robot_number(config_file)
    robot_ip = '192.168.11.' + str(robot_num)
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('network_configs')
    robot_yaml_path = os.path.join(package_path, "config", config_file)
    with open(robot_yaml_path, 'r') as f:
        robot_cfg = yaml.load(f)
    for robot in robot_cfg.keys():
        if robot_cfg[robot]['IP-address'] == robot_ip:
            robot_name = robot
    return robot_name


def add_modify_data_dbl(dbl, dbm):
    """ Insert new stuff into the db. When called without data, only the
    hearbeat gets updated """
    # Do a quick check
    assert isinstance(dbm, su.DBMessage)
    assert isinstance(dbl, su.DBwLock)

    # Acquire lock and commit into db
    dbl.lock.acquire()
    # Create and store things in db
    if dbm.robot not in dbl.db:
        dbl.db[dbm.robot] = {}
    if dbm.feature_name not in dbl.db[dbm.robot]:
        dbl.db[dbm.robot][dbm.feature_name] = {}
    dbl.db[dbm.robot][dbm.feature_name]['dtype'] = dbm.dtype
    dbl.db[dbm.robot][dbm.feature_name]['priority'] = dbm.priority
    dbl.db[dbm.robot][dbm.feature_name]['ts'] = dbm.ts
    dbl.db[dbm.robot][dbm.feature_name]['data'] = dbm.data
    dbl.db[dbm.robot][dbm.feature_name]['ack'] = dbm.ack
    packed_data = su.pack_data(dbm)
    checksum_data = hash_comm.Hash(packed_data).digest()
    dbl.db[dbm.robot][dbm.feature_name]['hash'] = checksum_data
    dbl.lock.release()
    return checksum_data


def serialize_ros_msg(msg):
    # TODO check that we are not entering garbage
    sio_h = io.BytesIO()
    msg.serialize(sio_h)
    return sio_h.getvalue()


def deserialize_ros_msg(s_msg, msg_type):
    msgd = msg_type()
    msgd.deserialize(s_msg)
    return msgd


def parse_answer(api_answer):
    constructor = MSG_TYPES[api_answer.msg_type_hash]['obj']
    msg = constructor()
    msg.deserialize(api_answer.msg_content)
    ts = api_answer.timestamp
    feature_name = api_answer.msg_name
    ack = api_answer.ack
    return feature_name, ts, msg, ack
