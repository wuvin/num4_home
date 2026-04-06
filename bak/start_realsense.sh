#!/bin/bash

# ./run_container_ros2.sh -v -y realsense -c /local/config_realsense_nomadgwick.yaml

./run_container_ros2.sh -v -y realsense --madgwick -l /local/launch_realsense_madgwick.py
echo "Restoring original version of launch file"
sudo cp ~/new_docker_test/m4-sensors/src/realsense2_bringup/launch/d435i_with_madgwick.launch.py ~/new_docker_test/m4-sensors/install/realsense2_bringup/share/realsense2_bringup/launch/d435i_with_madgwick.launch.py
