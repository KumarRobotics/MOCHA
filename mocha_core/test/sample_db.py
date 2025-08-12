#!/usr/bin/env python3
import copy
from pprint import pprint
import sys
sys.path.append('..')
import hash_comm
import database as db
import database_utils as du
import pdb
import rospy

DB_TEMPLATE = {
        0: {
            0: {
                'msg1': {
                    'dtype': 1,
                    'priority': 3,
                    'ts': 115.2223,
                    'data': bytes('Binary chunk', 'utf-8'),
                    },
                'msg2': {
                    'dtype': 2,
                    'priority': 0,
                    'ts': 117.2223,
                    'data': bytes('Binary chunk', 'utf-8'),
                    },
                },
            1: {
                'msg3': {
                    'dtype': 3,
                    'priority': 1,
                    'ts': 118.2223,
                    'data': bytes('Binary chunk', 'utf-8'),
                    },
                'msg4': {
                    'dtype': 3,
                    'priority': 1,
                    'ts': 119.2223,
                    'data': bytes('Binary chunk', 'utf-8'),
                    },
                },
            },
        1: {
            0: {
                'msg1': {
                    'dtype': 1,
                    'priority': 4,
                    'ts': 335.22,
                    'data': bytes('Binary chunk', 'utf-8'),
                    },
                'msg2': {
                    'dtype': 2,
                    'priority': 2,
                    'ts': 335.2,
                    'data': bytes('Binary chunk', 'utf-8'),
                    },
                }
            }
        }


def get_sample_dbl():
    DB = {}
    for robot_id in DB_TEMPLATE:
        if robot_id not in DB:
            DB[robot_id] = {}
        for topic_id in DB_TEMPLATE[robot_id]:
            if topic_id not in DB[robot_id]:
                DB[robot_id][topic_id] = {}
            for msg in DB_TEMPLATE[robot_id][topic_id]:
                ts = DB_TEMPLATE[robot_id][topic_id][msg]['ts']
                ts = rospy.Time.from_sec(ts)
                data = DB_TEMPLATE[robot_id][topic_id][msg]['data']
                dtype = DB_TEMPLATE[robot_id][topic_id][msg]['dtype']
                prio = DB_TEMPLATE[robot_id][topic_id][msg]['priority']
                c = db.DBMessage(robot_id, topic_id, dtype, prio, ts, data)
                DB[robot_id][topic_id][c.header] = c
    dbl = db.DBwLock(DB)
    return dbl


if __name__ == '__main__':
    dbl = get_sample_dbl()
    print(dbl)
