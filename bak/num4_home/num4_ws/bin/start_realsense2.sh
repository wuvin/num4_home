#!/bin/bash

source "${PWD}"/install/setup.bash
ros2 launch realsense2_bringup launch_d455.py config_file:=config.yaml use_imu_filter:=false
