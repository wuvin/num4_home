#!/bin/bash

MSG="source /opt/ros/humble/setup.bash"
MSG+=" && ros2 run tf2_ros static_transform_publisher"
MSG+=" 0 0 0"  # translational offset
#MSG+=" 0 0 3.14159"  # camera_link livox_frame rotational offset, ypr rad
#MSG+=" camera_link livox_frame"
MSG+=" -1.570796 -1.570796 0"  # camera_color_optical_frame livox_frame
MSG+=" camera_color_optical_frame livox_frame"
docker exec -it num4_ros2 /bin/bash -c "${MSG}"
