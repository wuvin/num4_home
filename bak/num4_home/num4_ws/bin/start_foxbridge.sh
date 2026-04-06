#!/bin/bash

source "${PWD}"/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml send_buffer_limit:=20000000
