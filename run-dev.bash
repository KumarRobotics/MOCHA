#!/bin/bash

xhost +

docker run --rm -it \
  --network=host \
  --privileged \
  --entrypoint="" \
  -e MOCHA=true \
  --name dtc-platform-`hostname`-mocha \
  dtc-platform-`hostname`:mocha \
  bash

xhost -
