#!/bin/bash
set -e

# source base lib
source $BASE_LIB_WS_SETUP
# setup ros environment
source $ROS_WS_SETUP
# setup sonia environment
source $SONIA_WS_SETUP

exec "$@"
