#!/bin/bash

# Shows examples of commands executed inside a Docker container for some
# specific ROS 2 functionality. Refer to `run_num4_container.sh` for how
# to run a Docker container in which these commands are executed.
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: February 18, 2026

# Foxglove Bridge
printf "To run Foxglove bridge:\n"
printf "\tsource \"\${PWD}\"/install/setup.bash\n"
printf "\tros2 launch foxglove_bridge foxglove_bridge_launch.xml" 
printf " send_buffer_limit:=2000000\n"

# Livox
printf "\nTo run Livox bringup:\n"
printf "\tsource \"\${PWD}\"/install/setup.bash\n"
printf "\tros2 launch livox_bringup launch_mid360.py" 
printf " config_file:=config_norepeat.json"
printf " xfer_format:=1"
printf " publish_freq:=50.0\n"

# RealSense
printf "\nTo run RealSense bringup:\n"
printf "\tsource \"\${PWD}\"/install/setup.bash\n"
printf "\tros2 launch realsense2_bringup launch_d455.py" 
printf " config_file:=config.yaml"
printf " use_imu_filter:=false\n"

# Static transform publisher
printf "\nTo relate the optical and Livox frames:\n"
printf "\tsource \"\${PWD}\"/install/setup.bash\n"
printf "\tros2 run tf2_ros static_transform_publisher" 
printf " 0 0 0"  # translation
printf " -1.5708 -1.5708 0"  # rpy, radians
printf " camera_color_optical_frame livox_frame\n"

# Point cloud custom message format
printf "\nTo convert Livox custom messages into PointCloud2 format:\n"
printf "\tsource \"\${PWD}\"/install/setup.bash\n"
printf "\tros2 run livox_to_pointcloud2 livox_to_pointcloud2_node\n"

# Point cloud to image
printf "\nTo project point cloud scans onto image plane:\n"
printf "\tsource \"\${PWD}\"/install/setup.bash\n"
printf "\tros2 launch pc2image launch_projector.py\n"