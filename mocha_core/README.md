mocha_core
====================

The distributed database allows sending messages asynchronously between the
ground station and the robots.

What does each file do?
----------------------

- `config/`: configuration folder with yaml files. Configure here IPs,
  ports, and offsets.
- `scripts/hash_comm.py`: utilities for hashing messages and data
  transmitted.
- `scripts/zmq_comm_node.py`: client/server architecture connecting two
  nodes, based on ZeroMQ.
- `scripts/synchronize_channel.py`: database synchronization between two
  nodes.
- `scripts/test/`: unit test files for previous scripts.


What is in the database
------------------------

The database is composed of messages, each of which has the following
fields:

 - Priority: 3 to 0, being 3 the higher priority.
 - Timestamp: in local time of each roboteach robot.
 - Message type: one of the following:
   - Priority 3 - Robot State ( x 1 ) : Pose(geometry_msgs/Pose) + State(std_msgs/Int8 : 1. Explore, 2. Stop, 3. MoveFrontier, 4. MoveBase)
   - Priority 3 - Object Detection ( x n ) : nearest graph node(std_msgs/Int16) + pose(vision__msgs/ObjectHypothesisWithPose)
   - Priority 2 - Topological Graph ( x 1 ) : an array indicating edge-node relation(std_msgs/ByteMultiArray)
   - Priority 1 - (Pseudo) Frontiers ( x n ) : nearest graph node(std_msgs/Int16) + frontier direction(geometry_msgs/Vector3)
   - Priority 0 - Panaramic Images ( x n ) : compressed depth image(std_msgs/Uint16MultiArray)
   - default - heartbeat(std_msgs/Header)
 - Binary data
 - Hash

IMPORTANT: All the incremental information that goes into the database
has to be filtered. The database runs currently on RAM, so THERE IS a
hard limit on the number of information each robot may store (around 200
MB).

Message protocol
----------------
The message protocol is specified in `synchronize_utils.py`.
