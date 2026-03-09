#!/bin/bash

docker build --rm -t dtc-platform-$(hostname):mocha -f Dockerfile.x86 .
