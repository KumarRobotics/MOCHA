#!/bin/bash

HOSTNAME=$(hostname)

UGVS=("phobos" "deimos" "titania" "oberon" "aphrodite" "ares")
UAVS=("dione")
BASESTATIONS=("neptune" "turing")

source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash
if [ "${MOCHA}" == "true" ]; then
    if [[ " ${UGVS[*]} " == *" $HOSTNAME "* ]]; then
        ros2 launch mocha_launch jackal.launch.py robot_name:=$HOSTNAME
    
    elif [[ " ${UAVS[*]} " == *" $HOSTNAME "* ]]; then
        ros2 launch mocha_launch titan.launch.py robot_name:=$HOSTNAME
    
    elif [[ " ${BASESTATIONS[*]} " == *" $HOSTNAME "* ]]; then
        ros2 launch mocha_launch basestation.launch.py robot_name:=basestation
    
    else
        echo "Error: Hostname '$HOSTNAME' not recognized."
        exit 1
    fi
fi
