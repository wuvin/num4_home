#!/bin/bash

CHANGE_ME=42

source /opt/ros/humble/setup.bash

ros2 topic list
echo "Resetting ROS_DOMAIN_ID from [${ROS_DOMAIN_ID}] to [${CHANGE_ME}]..."
export ROS_DOMAIN_ID="$CHANGE_ME"
ros2 topic list
