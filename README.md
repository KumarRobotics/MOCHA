DCIST Distributed DB Communications
-----------------------------------

This repository contains the distributed communication stack developed by the
GRASP Lab at UPenn.

Directories
===========
 - `comm_stack_launch/`: launch files for the communication stack. These will
 - `distributed_database/`: the distributed database itsel.
 - `hal_rajant/`: Rajant specific modules.
   into the distributed database.
   launch all the required nodes in the other directories.
   The current method uses a polling mechanism.
 - `topic_publisher`: node to publish messages from the distributed database.
 - `translators/`: the translators listen for specific messages and insert them

Dependencies:
=============

```
sudo apt update
pip3 install rospkg
pip3 install defusedxml
sudo apt install python3-zmq
```

Questions?
==========

Please do not hesitate to contact:

Fernando Cladera [fclad@seas.upenn.edu](fclad@seas.upenn.edu)
