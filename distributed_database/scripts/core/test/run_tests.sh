#!/bin/bash
set -eo pipefail

# Echo some variables for debug
echo "$PYTHONPATH"

# Colors for better distinguishing start and end of tests
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# TESTS_TO_RUN is a list of tests that should be executed in the CI
TESTS_TO_RUN="test_database_utils.py test_database.py test_zmq_comm_node.py"

# The first argument is the path to the catkin workspace + catkin_ws
CATKIN_WS="$1/catkin_ws"
# Display path
echo "Using catkin workspace: $CATKIN_WS"

# Source the ROS environment
source "/$CATKIN_WS/devel/setup.bash"

# Run roscore in the background and save its PID to kill it later
roscore > /dev/null 2>&1 &
PID=$(ps -ef | grep roscore | grep -v grep | awk '{print $2}')
echo "Waiting 2 seconds for roscore to load"
sleep 2

# TODO(fclad) Change this when all the tests are passing
# Run all the files that start with test_ in this folder
# for i in $(ls | grep "^test_"); do

for i in $TESTS_TO_RUN; do
  # Print yellow separator
  echo -e "${YELLOW}===================================== $i - START${NC}"

    rosrun distributed_database "$i"
    retVal=$?
    sleep 5 # To wait for some spawned processes
    if [ $retVal -ne 0 ]; then
      echo -e "${RED}${BOLD}===================================== $i - FAILED${NC}"
      kill -9 "$PID"
      exit $retVal
    fi
done
echo -e "${GREEN}${BOLD}================================ALL TESTS PASSED${NC}"
exit 0
