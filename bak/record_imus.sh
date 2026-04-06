#!/bin/bash

bagname=

if [[ -z "${bagname}" ]]; then
    read -p "Enter bag name to begin recording:  " bagname
fi

./run_container_ros2.sh -v record -o "/local/data/${bagname}" -s mcap -t imus
