#!/usr/bin/env python3
import copy
from pprint import pprint
import sys
sys.path.append('..')
import hash_comm
import database as db
import database_utils as du

DB_TEMPLATE = {
        0: {
            'feature1': {
                'dtype': 1,
                'priority': 3,
                'hash': None,
                'ts': 115.2223,
                'signature': False,
                'data': bytes('Binary chunk', 'utf-8'),
                'ack': False
                },
            'feature2': {
                'dtype': 2,
                'priority': 0,
                'hash': None,
                'ts': 117.2223,
                'signature': False,
                'data': bytes('Binary chunk', 'utf-8'),
                'ack': False
                },
            'feature3': {
                'dtype': 3,
                'priority': 1,
                'hash': None,
                'ts': 118.2223,
                'signature': False,
                'data': bytes('Binary chunk', 'utf-8'),
                'ack': False
                },
            'feature4': {
                'dtype': 3,
                'priority': 1,
                'hash': None,
                'ts': 119.2223,
                'signature': False,
                'data': bytes('Binary chunk', 'utf-8'),
                'ack': False
                },
            },
        1: {
            'feature1': {
                'dtype': 1,
                'priority': 4,
                'hash': None,
                'ts': 335.22,
                'signature': False,
                'data': bytes('Binary chunk', 'utf-8'),
                'ack': False
                },
            'feature2': {
                'dtype': 2,
                'priority': 2,
                'hash': None,
                'ts': 335.2,
                'signature': False,
                'data': bytes('Binary chunk', 'utf-8'),
                'ack': False
                },
            }
        }


def get_sample_dbl():
    DB = copy.deepcopy(DB_TEMPLATE)
    for robot in DB:
        for feature in DB[robot]:
            ts = DB[robot][feature]['ts']
            data = DB[robot][feature]['data']
            dtype = DB[robot][feature]['dtype']
            prio = DB[robot][feature]['priority']
            ack = DB[robot][feature]['ack']
            c = db.DBMessage(robot, feature, dtype, prio, ts, data, ack)
            packed = du.pack_data(c)
            checksum = hash_comm.Hash(packed).digest()
            DB[robot][feature]['hash'] = checksum
    dbl = db.DBwLock(DB)
    return dbl
