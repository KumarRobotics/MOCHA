#!/bin/bash

# Echo some variables for debug
echo "$PYTHONPATH"

# Run roscore in the background (and wait until it is running)
roscore > /dev/null 2>&1 &
echo "Waiting 2 seconds for roscore to load"
sleep 2

# Run tests in a loop
for i in $(ls | grep "^test_"); do
    echo "===================================== $i"
    rosrun distributed_database "$i"
    retVal=$?
    if [ $retVal -ne 0 ]; then
        echo "Error"
        exit $retVal
    fi
done
exit 0
