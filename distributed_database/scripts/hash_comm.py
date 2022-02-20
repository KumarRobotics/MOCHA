#!/usr/bin/env python3
import hashlib

# TODO: we may want the hash not to be hexadecimal, but other type to
# reduce bandwith

class Hash():
    HASH_LENGTH_HEX = 12
    # Length of the hash in bits
    # We can expect a collision after approx
    # math.sqrt(2**HASH_LENGTH_BITS)
    # https://preshing.com/20110504/hash-collision-probabilities/
    HASH_LENGTH_BITS = HASH_LENGTH_HEX*4
    HASH_LENGTH = HASH_LENGTH_HEX

    def __init__(self, data):
        if type(data) != bytes:
            raise TypeError
        self.data = data

    def digest(self):
        h = hashlib.sha256(self.data)
        hexdigest = h.hexdigest()
        trimmed_digest = hexdigest[:self.HASH_LENGTH_HEX]
        return trimmed_digest

    def bindigest(self):
        return self.digest().encode()


if __name__ == "__main__":
    import random
    import numpy as np
    MAX_DATA = 10000000
    collision_i = np.array([])
    for loop in range(100):
        hashes = set()
        for i in range(MAX_DATA):
            randstr = str(random.random())
            hi = Hash(randstr.encode()).digest()
            if hi in hashes:
                print(loop, "- Collision on hash %d -" % i, hi)
                collision_i = np.append(collision_i, i)
                break
            hashes.add(hi)
    print("Avg collision: %f" % np.average(collision_i))
