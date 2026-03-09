#!/bin/bash

docker build --network=host --rm -t dtc-platform-$(hostname):mocha -f Dockerfile.l4t .
