#!/bin/bash

docker build --rm --no-cache -t dtc-platform-$(hostname):mocha -f Dockerfile.x86 .
