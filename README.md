☕ MOCHA: Multi-robot Opportunistic Communication for Heterogeneous Collaboration
---------------------------------------------------------------------------------
![MOCHA gif](mocha.gif)

This repository contains the distributed and opportunistic communication stack used for multi-robot experiments at KumarRobotics.

## Directories

 - `mocha_launch/`: launch files the different robots in MOCHA. The launch file
   sets up the `robot_name` argument,
 - `mocha_core/`: MOCHA's main components (source code, config files).
 - `interface_rajant/`: interface for Rajant breadrumb radios

## Dependencies:

MOCHA requires `rospkg`, `defusedxml`, and `python3-zmq`. You may install these
packages with:

```
sudo apt update
pip3 install rospkg
pip3 install defusedxml
sudo apt install python3-zmq
```

## Contribution - Questions

Please [fill-out an issue](https://github.com/KumarRobotics/MOCHA/issues) if you have any questions.
Do not hesitate to [send your pull request](https://github.com/KumarRobotics/MOCHA/pulls).

## Citation

If you find MOCHA useful, please cite:

```
@misc{cladera2023mocha,
      title={Enabling Large-scale Heterogeneous Collaboration with Opportunistic Communications}, 
      author={Fernando Cladera, Zachary Ravichandran, Ian D. Miller, M. Ani Hsieh, C. J. Taylor, and Vijay Kumar}
}
