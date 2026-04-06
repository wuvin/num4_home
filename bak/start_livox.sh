#!/bin/bash

./run_container_ros2.sh -v -y livox -l /local/launch_livox.py
echo "Restoring original version of launch file"
sudo cp ~/new_docker_test/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py ~/new_docker_test/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
