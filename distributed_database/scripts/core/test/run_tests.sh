#!/bin/bash

# Echo some variables for debug
echo "$PYTHONPATH"

# Colors for better distinguishing start and end of tests
COLOR='\033[0;35m'
NC='\033[0m' # No Color

# Run roscore in the background (and wait until it is running)
roscore > /dev/null 2>&1 &
echo "Waiting 2 seconds for roscore to load"
sleep 2

# Run all the files that start with test_ in this folder
for i in $(ls | grep "^test_"); do
    echo -e "${COLOR}===================================== $i - START${NC}"
    rosrun distributed_database "$i"
    retVal=$?
    if [ $retVal -ne 0 ]; then
        echo "Error"
        exit $retVal
    fi
    sleep 1 # To wait for some spawned processes
done
echo -e "${COLOR}===================================== $i - END${NC}"
echo "ALL TESTS PASSED"
exit 0
