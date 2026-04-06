#!/bin/bash

#======================
# SCRIPT CONFIGURATION
#======================

# ROS2 parameters
DOMAIN_ID=42
RMW_PROTOCOL=rmw_cyclonedds_cpp

# Docker
IMAGE_NAME="num4-wuvin-base:v2"
CONTAINER_NAME="num4_ros2"

# Pathing
HOST_HOME="/home/num4/new_docker_test/m4_home" #"$(realpath /home/num4/new_docker_test/m4_home)"
#HOST_HOME="$(realpath /home/m4_home)"
CTNR_HOME="/home/num4"

HOST_SRC="/home/num4/new_docker_test" #"$(realpath /home/num4/new_docker_test)" 
#HOST_SRC="$(realpath /home/num4)"
CTNR_SRC="$CTNR_HOME/m4_ws/src"

#=================
# CLI FLAG INPUTS
#=================

# Set display outputs for help flag
function print_usage() {
  printf "usage: $0 [-h] [-v] [-y]\n"
  printf "       %*s " "${#0}" ""
  printf "Call \`$0 <command>\` -h for more detailed usage. ...\n"
  printf "\n"
  printf "Launches ROS 2 Humble container\n"
  printf "\n"
  printf "options:\n"
  printf "  -h, --help    Show this help message and exit\n"
  printf "  -v            Enable verbosity\n"
  printf "  -y            Skip any [y]es/[n]o prompts\n"
  printf "\n"
  printf "commands:\n"
  printf "  foxbridge  Launches foxglove_bridge for data visualization\n"
  printf "  realsense  Runs camera node for RealSense D455 RGB-D + IMU\n"
  printf "  livox      Launches Livox driver for LiDAR + IMU\n"
  printf "  record     Opens a terminal with a ros2 bag recording\n"
  printf "\n"
  printf "  Call \`$0 <command>\` -h for more detailed usage.\n"
  exit 0
}

function print_command_usage() {
  if [[ "$1" == "foxbridge" || "$1" == "foxglove" ]]; then
    printf "usage: $0 foxbridge [-h] [--buffer-limit BYTES]\n"
    printf "\n"
    printf "Connect ROS stack to Foxglove\n"
    printf "\n"
    printf "options:\n"
    printf "  -h, --help    Show this help message and exit\n"
    printf "  -b BYTES, --buffer-limit BYTES\n"
    printf "                Increase buffer size (default 10 MB)\n"
    printf "  -p PORT, --port PORT\n"
    printf "                Port number (default 8765)\n"
    exit 0

  elif [[ "$1" == "realsense" ]]; then
    printf "usage: $0 realsense [-h] [--madgwick] [--config FILE]"
    printf "\n"
    printf "Runs driver for RealSense D455 camera and IMU\n"
    printf "\n"
    printf "options:\n"
    printf "  -h, --help    Show this help message and exit\n"
    printf "  --madgwick    Enable Madgwick filter for IMU\n"
    printf "  -c FILE, --config FILE\n"
    printf "                Run using a YAML to set parameters\n"
    printf "  -l FILE, --launch FILE\n"
    printf "                Replace install/ copy of launch file for changes\n"
    exit 0

  elif [[ "$1" == "livox" ]]; then
    printf "usage: $0 livox [-h] [--lidar-rate RATE]\n"
    printf "\n"
    printf "Runs driver for Livox LiDAR and IMU\n"
    printf "\n"
    printf "options:\n"
    printf "  -h, --help    Show this help message and exit\n"
    printf "  -l FILE, --launch FILE\n"
    printf "                Replace install/ copy of launch file for changes\n"
    exit 0

  elif [[ "$1" == "record" ]]; then
    printf "usage: $0 record [-h] [-o OUTPUT] [-s FORMAT] [-t TOPICS]\n"
    printf "\n"
    printf "Initiate a recording of ROS 2 topic data\n"
    printf "\n"
    printf "options:\n"
    printf "  -h, --help    Show this help message and exit\n"
    printf "  -o OUTPUT, --output OUTPUT\n"
    printf "                Destination of bagfile to create\n"
    printf "  -s FORMAT, --storage FORMAT\n"
    printf "                Identifier of storage to use, default 'mcap'\n"
    printf "  -t TOPICS, --topics TOPICS\n"
    printf "                Topic(s) and/or topic group(s) to record\n"
    exit 0
  fi
}

# Set default flags
VERBOSE=false
SKIP=false

# Parse flags
while getopts "hvy-" opt; do
  case $opt in
    h) print_usage ;;
    v) VERBOSE=true ;;
    y) SKIP=true ;;
    -)
      chars=""
      count=0
      while [[ $count < 4 ]]; do
        getopts help addtl
        chars+=$addtl
        ((count+=1))
      done

      if [[ "$chars" != "help" ]]; then
        printf "$0: illegal option -- $chars (expected --help)\n"
        exit 1
      fi
      
      print_usage
      ;;
    \?)
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1))  # shift for positional arguments w/o flag

# Treat remaining positional arguments as command and its arguments
COMMAND=$1
COMMAND_ARGS=("${@:2}")

# Define strings to denote common groups of topics
function parse_topics() {
  local input="$1"
  if [[ "$input" == "imus" ]]; then
    echo "/rs_imu/data /livox/imu "
  elif [[ "$input" == "imus_sample" ]]; then
    echo "/camera/camera/accel/sample /camera/camera/gyro/sample "
  elif [[ "$input" == "rgb" || "$input" == "color" ]]; then
    TOPICS="/camera/camera/color/image_raw "
    TOPICS+="/camera/camera/color/camera_info "
    echo "$TOPICS"
  elif [[ "$input" == "aligned_depth" ]]; then
    TOPICS="/camera/camera/aligned_depth_to_color/image_raw "
    TOPICS+="/camera/camera/aligned_depth_to_color/camera_info "
    echo "$TOPICS"
  elif [[ "$input" == "lidar" ]]; then
    echo "/livox/lidar "
  else
    echo "$input "
  fi
}

#================
# PRE-PROCESSING
#================

# Verification in terminal
if [[ "$VERBOSE" == "true" ]]; then
  printf "Configuration for ${0}:\n"
  printf "\tIMAGE:      ${IMAGE_NAME}\n"
  printf "\tCONTAINER:  ${CONTAINER_NAME}\n"
  printf "\n"
  printf "\t${HOST_HOME}  >  ${CTNR_HOME}\n"
  printf "\t${HOST_SRC}  >  ${CTNR_SRC}\n"
  printf "\n"
  printf "\tROS_DOMAIN_ID:       ${DOMAIN_ID}\n"
  printf "\tRMW_IMPLEMENTATION:  ${RMW_PROTOCOL}\n"

  if [[ "$SKIP" != "true" ]]; then
    read -p "Do you want to continue ([y]es/[n]o)?  " user_confirm
  else
    user_confirm="y"
  fi
  if [[ "${user_confirm,,}" != "y" && "${user_confirm,,}" != "yes" ]]; then
    echo "User early exit."
    exit 0
  fi
  echo
fi

# Print usage help for specific commands
for arg in "${COMMAND_ARGS[@]}"; do
  if [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
    print_command_usage $COMMAND
  fi
done

# Get commands to execute in terminal upon running container
if [[ "$COMMAND" == "foxbridge" || "$COMMAND" == "foxglove" ]]; then  
  count=0
  buffer_limit=false
  port=false
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "--buffer-limit" || "$arg" == "-b" ]]; then
      ((count+=1)) && buffer_limit=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "--port" || "$arg" == "-p" ]]; then
      ((count+=1)) && port=${COMMAND_ARGS[$count]}
    else
      printf "$0 foxbridge: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  if [[ "$VERBOSE" == "true" ]]; then
    echo "Pre-loading foxglove bash commands to execute in terminal..."
  fi

  RUN_CMD="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
  if [[ "$port" != "false" ]]; then
    RUN_CMD+=" port:=${port}"
  fi
  if [[ "$buffer-limit" != "false" ]]; then
    RUN_CMD+=" send_buffer_limit:=${buffer_limit}"
  fi

  CMD="(
        echo \"export ROS_DOMAIN_ID=${DOMAIN_ID}\" >> ~/.bashrc
        echo \"export RMW_IMPLEMENTATION=${RMW_PROTOCOL}\" >> ~/.bashrc
        source /opt/ros/humble/setup.bash
        ${RUN_CMD}
        /bin/bash
       )"

elif [[ "$COMMAND" == "realsense" ]]; then
  count=0
  use_madgwick=false
  config_file=false
  launch_file=false
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "--madgwick" ]]; then
      use_madgwick=true
    elif [[ "$arg" == "-c" || "$arg" == "--config" ]]; then
      ((count+=1)) && config_file=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "-l" || "$arg" == "--launch" ]]; then
      ((count+=1)) && launch_file=${COMMAND_ARGS[$count]}
    else
      printf "$0 realsense: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  if [[ "$VERBOSE" == true ]]; then
    echo "Pre-loading realsense bash commands to execute in terminal..."
  fi

  ADD_CMD="cd ${CTNR_SRC}/m4-sensors"
  if [[ "$launch_file" != "false" && "$use_madgwick" == "false" ]]; then
    printf "$0 realsense: use config instead of launch without madgwick\n"
    exit 1
  elif [[ "$launch_file" != "false" ]]; then
    LAUNCH_DIR=install/realsense2_bringup/share/realsense2_bringup/launch/
    TARGET_FILE="${LAUNCH_DIR}d435i_with_madgwick.launch.py"
    ADD_CMD+=" && mv ${TARGET_FILE} ${TARGET_FILE}.bak"  # backup
    ADD_CMD+=" && cp ${launch_file} ${TARGET_FILE}"  # replace
  fi

  RUN_CMD="ros2 "
  if [[ "$use_madgwick" == "true" ]]; then
    RUN_CMD+="launch realsense2_bringup d435i_with_madgwick.launch.py"
  else
    RUN_CMD+="run realsense2_camera realsense2_camera_node"
  fi
  if [[ "$config_file" != "false" && "$use_madgwick" == "true" ]]; then
    printf "$0 realsense: use launch instead of config with madgwick\n"
    exit 1
  elif [[ "$config_file" != "false" ]]; then
    RUN_CMD+=" --ros-args --params-file ${config_file}"
  fi

  CMD="(
        echo \"export ROS_DOMAIN_ID=${DOMAIN_ID}\" >> ~/.bashrc
        echo \"export RMW_IMPLEMENTATION=${RMW_PROTOCOL}\" >> ~/.bashrc
        ${ADD_CMD}
        source install/setup.bash
        ${RUN_CMD}
        /bin/bash
       )"
       
elif [[ "$COMMAND" == "livox" ]]; then
  count=0
  launch_file=false
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "--launch-file" || "$arg" == "-l" ]]; then
      ((count+=1)) && launch_file=${COMMAND_ARGS[$count]}
    else
      printf "$0 livox: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  if [[ "$VERBOSE" == true ]]; then
    echo "Pre-loading livox bash commands to execute in terminal..."
  fi

  ADD_CMD="cd ${CTNR_SRC}/ws_livox"
  if [[ "$launch_file" != "false" ]]; then
    LAUNCH_DIR=install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/
    TARGET_FILE="${LAUNCH_DIR}msg_MID360_launch.py"
    ADD_CMD+=" && mv ${TARGET_FILE} ${TARGET_FILE}.bak"  # backup
    ADD_CMD+=" && cp ${launch_file} ${TARGET_FILE}"  # replace
  fi

  CMD="(
        echo \"export ROS_DOMAIN_ID=${DOMAIN_ID}\" >> ~/.bashrc
        echo \"export RMW_IMPLEMENTATION=${RMW_PROTOCOL}\" >> ~/.bashrc
        ${ADD_CMD}
        source install/setup.bash
        ros2 launch livox_ros_driver2 msg_MID360_launch.py
        /bin/bash
       )"

elif [[ "$COMMAND" == "record" ]]; then
  count=0
  output_dir=""
  storage_id="mcap"
  topics=""
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "-o" || "$arg" == "--output" ]]; then
      ((count+=1)) && output_dir=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "-s" || "$arg" == "--storage" ]]; then
      ((count+=1)) && storage_id=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "-t" || "$arg" == "--topics" ]]; then
      ((count+=1)) && arg="${COMMAND_ARGS[$count]}"
      while [[ ! "${arg}" =~ ^[-?] && -n "${arg}" ]]; do
        topics+=$(parse_topics "${arg}")
        ((count+=1)) && arg="${COMMAND_ARGS[$count]}"
      done
      topics="${topics% }"  # remove trailing space
      ((count-=1))  # revisit arg
    elif [[ ! "$arg" =~ ^[-?] ]]; then
      while [[ ! "${arg}" =~ ^[-?] && -n "${arg}" ]]; do
        topics+=$(parse_topics "${arg}")
        ((count+=1)) && arg="${COMMAND_ARGS[$count]}"
      done
      topics="${topics% }"  # remove trailing space
      ((count-=1))  # revisit arg
    elseSCRIPT_DIR="$(dirname -- "${BASH_SOURCE[0]}")"
      printf "$0 record: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  if [[ -z "$topics" ]]; then
    if [[ "$VERBOSE" == "true" && "$SKIP" != "true" ]]; then
      read -p "Record all topics ([y]es/[n]o)?  " user_confirm
      echo
    else
      user_confirm="y"
    fi

    if [[ "${user_confirm,,}" == "y" || "${user_confirm,,}" == "yes" ]]; then
      topics+="-a"
    else
      echo "Then specify which topics to record."
      exit 1
    fi
  fi

  RUN_CMD="ros2 bag record -s ${storage_id}"
  if [[ -n "$output_dir" ]]; then
    RUN_CMD+=" -o ${output_dir}"
  fi
  RUN_CMD+=" ${topics}"

  if [[ "$VERBOSE" == true ]]; then
    echo "Pre-loading ros2 bag record bash commands to execute in terminal..."
  fi

  CMD="(
        echo \"export ROS_DOMAIN_ID=${DOMAIN_ID}\" >> ~/.bashrc
        echo \"export RMW_IMPLEMENTATION=${RMW_PROTOCOL}\" >> ~/.bashrc
        cd wuvin
        source /opt/ros/humble/setup.bash
        ${RUN_CMD}
        /bin/bash
       )"

else
  CMD="/bin/bash"
fi

#======
# MAIN
#======

SCRIPT_DIR="$(dirname -- "${BASH_SOURCE[0]}")"

if [[ "$VERBOSE" == "true" ]]; then
  printf "The folowing commands will be executed in the new terminal:\n"
  printf "$CMD\n"
fi
if [[ "$VERBOSE" == "true" && "$SKIP" != "true" ]]; then
  read -p "Do you want to continue ([y]es/[n]o)?  " user_confirm
else
  user_confirm="y"
fi
if [[ "${user_confirm,,}" != "y" && "${user_confirm,,}" != "yes" ]]; then
  echo "User early exit."
  exit 0
fi

# Check if container is already running
echo "Container name is ${CONTAINER_NAME}"

if [ $(docker ps -q -f name=$CONTAINER_NAME) ]; then
  if [[ "$VERBOSE" == "true" ]]; then
    printf "Container ${CONTAINER_NAME} is running. "
    printf "Executing another terminal...\n"
  fi

  # Execute another terminal
  docker exec -it "${CONTAINER_NAME}" /bin/bash -c "$CMD"
else
  echo "Creating container..."
   # Run the container
  docker run --runtime nvidia -it --rm --privileged --name ${CONTAINER_NAME} \
             --network=host \
             -v ${HOST_HOME}:${CTNR_HOME} \
             -v ${HOST_SRC}/m4-firmware:${CTNR_SRC}/m4-firmware \
             -v ${HOST_SRC}/m4-perception:${CTNR_SRC}/m4-perception \
             -v ${HOST_SRC}/m4-sensors:${CTNR_SRC}/m4-sensors \
             -v ${HOST_SRC}/vrpn_mocap:${CTNR_SRC}/vrpn_mocap \
             -v ${HOST_SRC}/ws_livox:${CTNR_SRC}/ws_livox \
             -v ${HOST_SRC}/fs_lio_ws:${CTNR_SRC}/fs_lio_ws \
             -v ${SCRIPT_DIR}:/local \
             -v /etc/timezone:/etc/timezone:ro -v /etc/localtime:/etc/localtime:ro \
  	           -v /dev:/dev \
   	           --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw" \
             -e DISPLAY=$DISPLAY \
             -v /tmp/.X11-unix:/tmp/.X11-unix \
             ${IMAGE_NAME} -c "${CMD}"
fi
