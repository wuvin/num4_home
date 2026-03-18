#!/bin/bash

CHANGE_ME=192.168.0.146

source /opt/ros/humble/setup.bash

ros2 topic list
echo "Resetting ROS_DISCOVERY_SERVER from [${ROS_DISCOVERY_SERVER}] to [${CHANGE_ME}]..."
export ROS_DISCOVERY_SERVER="$CHANGE_ME"
ros2 topic list
