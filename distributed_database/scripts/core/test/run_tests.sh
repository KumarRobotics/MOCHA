#!/bin/bash
set -euo pipefail

# Echo some variables for debug
echo "$PYTHONPATH"

# Colors for better distinguishing start and end of tests
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Execute all the commands of this script in the directory where the script is
# located
cd "$(dirname "${BASH_SOURCE[0]}")"

# Run roscore in the background and save its PID to kill it later
roscore > /dev/null 2>&1 &
PID=$(ps -ef | grep roscore | grep -v grep | awk '{print $2}')
echo "Waiting 2 seconds for roscore to load"
sleep 2

# Run all the files that start with test_ in this folder
for i in $(ls | grep "^test_"); do
  # Print yellow separator
  echo -e "${YELLOW}===================================== $i - START${NC}"

    rosrun distributed_database "$i"
    retVal=$?
    if [ $retVal -ne 0 ]; then
      echo -e "${RED}===================================== $i - FAILED${NC}"
      kill -9 $PID
      exit $retVal
    fi
    sleep 1 # To wait for some spawned processes
done
echo -e "${YELLOW}================================ALL TESTS PASSED${NC}"
exit 0
