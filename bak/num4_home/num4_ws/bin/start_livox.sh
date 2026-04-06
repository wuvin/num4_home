#!/bin/bash

source "${PWD}"/install/setup.bash

ros2 launch livox_bringup launch_mid360.py config_file:=config_norepeat.json xfer_format:=1 publish_freq:=50.0
