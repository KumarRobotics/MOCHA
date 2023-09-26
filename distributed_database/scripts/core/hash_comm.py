#!/usr/bin/env python3
import hashlib
import rospy
import struct

LENGTH = 6


class Hash():
    HASH_LENGTH = LENGTH
    # Length of the hash in bits
    # We can expect a collision after approx
    # math.sqrt(2**HASH_LENGTH_BITS)
    # https://preshing.com/20110504/hash-collision-probabilities/
    HASH_LENGTH_BITS = HASH_LENGTH*8

    def __init__(self, data):
        if type(data) != bytes:
            raise TypeError
        self.data = data

    def digest(self):
        h = hashlib.sha256(self.data)
        hexdigest = h.digest()
        trimmed_digest = hexdigest[:self.HASH_LENGTH]
        return trimmed_digest

    def bindigest(self):
        return self.digest()


class TsHeader():
    # Get length in bytes
    HEADER_LENGTH = LENGTH

    def __init__(self, *, robot_id=None, topic_id=None, secs=None, msecs=None):
        # You should not use this constructor to create the class. Use
        # from_data or from_header instead
        self.robot_id = robot_id
        self.topic_id = topic_id
        self.secs = secs
        self.msecs = msecs

    @classmethod
    def from_data(cls, robot_id, topic_id, time):
        # Robot ID is a 1-byte unsigned integer. Check type and value
        assert type(robot_id) == int and robot_id >= 0 and robot_id < 256
        # Same for topic ID
        assert type(topic_id) == int and topic_id >= 0 and topic_id < 256
        # time is a rospy time object
        assert type(time) == rospy.Time

        # The header will be comprised of 1 byte for the robot number, 1 byte
        # for the topic id, and 4 bytes for the time. The first 2 bytes of the
        # time are the secs, and the 2 last bytes are the ms of the message.
        # This means that we have ms resolution. We only handle positive times
        assert type(time.secs) == int and time.secs >= 0
        assert type(time.nsecs) == int and time.nsecs >= 0
        secs = time.secs % 65536
        msecs = time.nsecs // 1000000
        robot_id = robot_id
        topic_id = topic_id
        return cls(robot_id=robot_id, topic_id=topic_id,
                   secs=secs, msecs=msecs)

    @classmethod
    def from_header(cls, header):
        robot_id = struct.unpack("!B", header[0:1])[0]
        topic_id = struct.unpack("!B", header[1:2])[0]
        secs = struct.unpack("!H", header[2:4])[0]
        msecs = struct.unpack("!H", header[4:6])[0]
        return cls(robot_id=robot_id, topic_id=topic_id,
                   secs=secs, msecs=msecs)

    def bindigest(self):
        # Assemble the header by concatenating data, using struct
        b = struct.pack("!B", self.robot_id)
        b += struct.pack("!B", self.topic_id)
        b += struct.pack("!H", self.secs)
        b += struct.pack("!H", self.msecs)
        return b

    def get_id_and_time(self):
        assert self.robot_id is not None
        assert self.topic_id is not None
        assert self.secs is not None
        assert self.msecs is not None
        time = rospy.Time(self.secs, self.msecs*1000000)
        return self.robot_id, self.topic_id, time

if __name__ == "__main__":
    import random
    import numpy as np
    import pdb
    import sys

    import argparse
    ap = argparse.ArgumentParser()
    # Add an argument to decide which test to run
    ap.add_argument("-t", "--test", required=True, help="Test to run")
    args = vars(ap.parse_args())

    if args["test"] == "hash":
        expected_collision = np.sqrt(2**Hash.HASH_LENGTH_BITS)
        MAX_DATA = 10000000000
        collision_i = np.array([])
        for loop in range(10):
            hashes = set()
            for i in range(MAX_DATA):
                randstr = str(random.random())
                hi = Hash(randstr.encode()).digest()
                if hi in hashes:
                    print(loop, "- Collision on hash %d -" % i, hi)
                    collision_i = np.append(collision_i, i)
                    break
                hashes.add(hi)
                if i % 1000000 == 0:
                    # print with only 3 decimals
                    print(f"{loop} {i}" +
                          f"Expected rate {(i/expected_collision):.3f}")
        print("Avg collision: %f" % np.average(collision_i))
    elif args["test"] == "time":
        for i in range(100):
            random_robot = random.randint(0, 255)
            random_topic = random.randint(0, 255)
            random_time = rospy.Time(random.randint(0, 65536),
                                     random.randint(0, 1000000000))
            ts = TsHeader.from_data(random_robot, random_topic, random_time)
            bindigest = ts.bindigest()
            ts2 = TsHeader.from_header(bindigest)
            robot_id, topic_id, time = ts2.get_id_and_time()
            assert robot_id == random_robot
            assert topic_id == random_topic
            assert time.secs == random_time.secs
            assert np.abs(time.nsecs-random_time.nsecs) < 1000000
    else:
        sys.exit("Invalid test")

