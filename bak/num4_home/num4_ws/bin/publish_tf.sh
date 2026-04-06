#!/bin/bash

source "${PWD}"/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5708 -1.5708 0 camera_color_optical_frame livox_frame
