#!/bin/bash

# Executable script commands to run a Docker container within a terminal
# session. This must be (1) executed instead of sourced and (2) executed
# within the terminal session intended for the container.
#
# Settings can be configured either with flags in the command line or in
# the script itself. Execute with the `-h` flag for info on settings.
#
# See `show_ros2_examples.sh` for equivalent bash commands that would be
# executed within each terminal.
#
# Set-up for data collection with NUM4 requires multiple executions, and
# therefore multiple terminal instances or windows, of this script. This
# is handled at the level above the individual session SSHing into NUM4.
#
# See `run_num4_data_collection.sh` (within `.../num4_home/bin/` instead
# of `.../num4_home/num4_ws/bin/`) for examples of executing this script
# multiple times.
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: February 18, 2026

#=============================
# DEFAULT INPUT CONFIGURATION
#=============================

# ROS 2
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Docker
IMAGE_NAME_TAG="num4-wuvin:v0"
CONTAINER_NAME="num4_ws"

# Foxglove Bridge
MESSAGE_BUFFER=
FOXBRIDGE_PORT=

# RealSense
RS_LAUNCH=launch_d455.py
RS_CONFIG=config.yaml
RS_FILTER=true

# Livox
LIVOX_LAUNCH=launch_mid360.py
LIVOX_CONFIG=config.json
LIVOX_FORMAT=custom  # { (0 | PointCloud2), (1 | CustomMsg | custom) }
LIVOX_FREQHZ=10.0
LIVOX_FILTER=true

# Other CLI flags
IS_VERBOSE=true
IGN_PROMPT=false  # ignores y/n prompts
TERM_TITLE=

if [[ "$0" != "$BASH_SOURCE" ]]; then
    # Script is being sourced
    ret="return"
else
    # Script is being executed
    ret="exit"
fi


#=================
# CLI FLAG INPUTS
#=================

# Set display outputs for help flag
function print_usage {
  printf "usage: $0 [-h] [-v] [-y] [-d VENDOR] [--id ID]\n"
  printf "       %*s " "${#0}"
  printf "[--image IMAGE] [-n NAME] [-t TITLE]\n"
  printf "Call \`$0 <command>\` -h for more detailed usage. ...\n"
  printf "\n"
  printf "Launches ROS 2 Humble container\n"
  printf "\n"
  printf "options:\n"
  printf "  -h, --help    Show this help message and exit\n"
  printf "  -v            Enable verbosity\n"
  printf "  -y            Skip any [y]es/[n]o prompts\n"
  printf "  -d VENDOR, --dds VENDOR, --rmw VENDOR\n"
  printf "                Set DDS vendor for RMW implementation\n"
  printf "  --id ID\n"
  printf "                Set ROS 2 domain ID\n"
  printf "  --image IMAGE\n"
  printf "                Choose image to run container from\n"
  printf "  -n NAME, --name NAME\n"
  printf "                Assign name to container\n"
  printf "  -t TITLE, --title TITLE\n"
  printf "                Label terminal title\n"
  printf "\n"
  printf "commands:\n"
  printf "  foxbridge  Launches foxglove_bridge for data visualization\n"
  printf "  realsense  Runs camera node for RealSense D455 RGB-D + IMU\n"
  printf "  livox      Launches Livox driver for LiDAR + IMU\n"
  printf "  record     Runs a ros2 bag recording\n"
  printf "\n"
  printf "  Call \`$0 <command>\` -h for more detailed usage.\n"
  exit 0
}

function print_command_usage {
  if [[ "$1" == "foxbridge" || "$1" == "foxglove" ]]; then
    printf "usage: $0 foxbridge [-h] [--buffer-limit BYTES]\n"
    printf "\n"
    printf "Connect ROS stack to Foxglove\n"
    printf "\n"
    printf "options:\n"
    printf "  -h, --help    Show this help message and exit\n"
    printf "  -b BYTES, --buffer BYTES\n"
    printf "                Increase buffer limit (default 10 MB)\n"
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
    printf "  -l FILE, --launch FILE\n"
    printf "                Specify launch file in realsense2_bringup\n"
    printf "  -c FILE, --config FILE\n"
    printf "                Specify config file in realsense2_bringup\n"
    printf "  --madgwick    Enable Madgwick filter for IMU\n"
    exit 0

  elif [[ "$1" == "livox" ]]; then
    printf "usage: $0 livox [-h] [--lidar-rate RATE]\n"
    printf "\n"
    printf "Runs driver for Livox LiDAR and IMU\n"
    printf "\n"
    printf "options:\n"
    printf "  -h, --help    Show this help message and exit\n"
    printf "  -l FILE, --launch FILE\n"
    printf "                Specify launch file in livox_bringup\n"
    printf "  -c FILE, --config FILE\n"
    printf "                Specify config file in livox_bringup\n"
    printf "  -f FORMAT, --format FORMAT\n"
    printf "                Use to specify transfer format of message\n"
    printf "  -r RATE, --rate RATE\n"
    printf "                Set LiDAR sampling rate in Hz\n"
    printf "  --madgwick    Enable Madgwick filter for IMU\n"
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

# Process long flags
PROCESSED_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) print_usage ;;
    --dds=*|--rmw=*) RMW_IMPLEMENTATION="${1#*=}"; shift ;;
    --dds|--rmw) RMW_IMPLEMENTATION="$2"; shift 2 ;;
    --id=*) ROS_DOMAIN_ID="${1#*=}"; shift ;;
    --id) ROS_DOMAIN_ID="$2"; shift 2 ;;
    --image=*) IMAGE_NAME_TAG="${1#*=}"; shift ;;
    --image) IMAGE_NAME_TAG="$2"; shift 2 ;;
    --name=*) CONTAINER_NAME="${1#*=}"; shift ;;
    --name) CONTAINER_NAME="$2"; shift 2 ;;
    --title=*) TERM_TITLE="${1#*=}"; shift ;;
    --title) TERM_TITLE="$2"; shift 2 ;;
    --) shift; PROCESSED_ARGS+=("$@"); break ;;  # end of options
    --*) printf "$0: illegal option -- ${1#--}\n" >&2; exit 1 ;;
    *) PROCESSED_ARGS+=("$1"); shift ;;  # save short opts or positional
  esac
done

# Restore remaining arguments for short flags (getopts) and commands
set -- "${PROCESSED_ARGS[@]}"

# Process short flags with getopts
while getopts "hvyd:n:t:" opt; do
  case $opt in
    h) print_usage ;;
    v) IS_VERBOSE=true ;;
    y) IGN_PROMPT=true ;;
    d) RMW_IMPLEMENTATION="$OPTARG" ;;
    n) CONTAINER_NAME="$OPTARG" ;;
    t) TERM_TITLE="$OPTARG" ;;
    \?) exit 1 ;;
  esac
done
shift $((OPTIND - 1))  # shift for commands and positional arguments

# Treat remaining positional arguments as commands and their arguments
COMMAND=$1
COMMAND_ARGS=("${@:2}")

# Define strings to denote common groups of topics for recording
function get_topics_to_record() {
  local topic_list=$(ros2 topic list -t)
  local found_topics=()

  for x in "$@"; do
    local found_in_x=()

    # Check if line matches topic
    if echo "$topic_list" | grep -q "^${x} "; then
      found_in_x+=("$x")
    else  # check if line matches message type
      local match=$(echo "$topic_list" | grep " \[${x}\]$" | awk '{print $1}')

      if [[ -z "$match" ]]; then  # no match -> use base instead
        match=$(echo "$topic_list" | grep " \[.*/${x}\]$" | awk '{print $1}')
      fi  # otherwise, got match on full message type string

      while IFS= read -r topic; do
        [[ -n "$topic" ]] && found_in_x+=("$topic")
      done
    fi

    # Raise error if no matches found
    if [[ ${#found_in_x[@]} -eq 0 ]]; then
      echo "ERROR: No matches for $x"
      return 1
    else
      found_topics+=("${found_in_x[@]}")
    fi
  done

  # Return matches
  echo "$found_topics"
}

#================
# PRE-PROCESSING
#================

# Script execution check
if [[ "$0" != "$BASH_SOURCE" ]]; then  # script is being sourced
  echo "This script is intended to be executed instead of sourced."
fi

# Command to set terminal title later on
[[ -n "$TERM_TITLE" ]] && TITLE_CMD='printf "\\033]0;'$TERM_TITLE'\\a"'

# Verification in terminal
if [[ "$IS_VERBOSE" == "true" ]]; then
  printf "Configuration for ${0}:\n"
  printf "\tIMAGE:      ${IMAGE_NAME_TAG}\n"
  printf "\tCONTAINER:  ${CONTAINER_NAME}\n"
  printf "\n"
  printf "\tTERMINAL:   ${TERM_TITLE}\n"
  printf "\n"
  printf "\tROS_DOMAIN_ID:       ${ROS_DOMAIN_ID}\n"
  printf "\tRMW_IMPLEMENTATION:  ${RMW_IMPLEMENTATION}\n"

  if [[ "$IGN_PROMPT" != "true" ]]; then
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
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "--buffer" || "$arg" == "-b" ]]; then
      ((count+=1)) && MESSAGE_BUFFER=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "--port" || "$arg" == "-p" ]]; then
      ((count+=1)) && FOXBRIDGE_PORT=${COMMAND_ARGS[$count]}
    else
      printf "$0 foxbridge: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  LAUNCH_CMD="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
  if [[ -n "$FOXBRIDGE_PORT" ]]; then
    LAUNCH_CMD+=" port:=${FOXBRIDGE_PORT}"
  fi
  if [[ -n "$MESSAGE_BUFFER" ]]; then
    LAUNCH_CMD+=" send_buffer_limit:=${MESSAGE_BUFFER}"
  fi

  CMD="(
        source install/setup.bash
        ${TITLE_CMD}
        ${LAUNCH_CMD}
        /bin/bash
       )"

elif [[ "$COMMAND" == "realsense" ]]; then
  count=0
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "-l" || "$arg" == "--launch" ]]; then
      ((count+=1)) && RS_LAUNCH=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "-c" || "$arg" == "--config" ]]; then
      ((count+=1)) && RS_CONFIG=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "--madgwick" ]]; then
      RS_FILTER=true
    else
      printf "$0 realsense: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  LAUNCH_CMD="ros2 launch realsense2_bringup $RS_LAUNCH"
  if [[ -n "$RS_CONFIG" ]]; then
    LAUNCH_CMD+=" config:=$RS_CONFIG"
  fi
  if [[ -n "$RS_FILTER" ]]; then
    LAUNCH_CMD+=" use_imu_filter:=$RS_FILTER"
  fi

  CMD="(
        source install/setup.bash
        ${TITLE_CMD}
        ${LAUNCH_CMD}
        /bin/bash
       )"
       
elif [[ "$COMMAND" == "livox" ]]; then
  count=0
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "--launch" || "$arg" == "-l" ]]; then
      ((count+=1)) && LIVOX_LAUNCH=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "--config" || "$arg" == "-c" ]]; then
      ((count+=1)) && LIVOX_CONFIG=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "--format" || "$arg" == "-f" ]]; then
      ((count+=1)) && LIVOX_FORMAT=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "--rate" || "$arg" == "-r" ]]; then
      ((count+=1)) && LIVOX_FREQHZ=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "--madgwick" ]]; then
      LIVOX_FILTER=true
    else
      printf "$0 livox: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  LAUNCH_CMD="ros2 launch livox_bringup $LIVOX_LAUNCH"
  if [[ -n "$LIVOX_CONFIG" ]]; then
    LAUNCH_CMD+=" config:=$LIVOX_CONFIG"
  fi
  LIVOX_FORMAT="${LIVOX_FORMAT,,}"
  if [[ "$LIVOX_FORMAT" == "custommsg" || "$LIVOX_FORMAT" == "custom" ]]; then
    LIVOX_FORMAT=1
  elif [[ "$LIVOX_FORMAT" == "pointcloud2" ]]; then
    LIVOX_FORMAT=0
  fi
  if [[ -n "$LIVOX_FORMAT" ]]; then
    LAUNCH_CMD+=" xfer_format:=$LIVOX_FORMAT"
  fi
  if [[ -n "$LIVOX_FREQHZ" ]]; then
    LAUNCH_CMD+=" publish_freq:=$LIVOX_FREQHZ"
  fi
  if [[ -n "$LIVOX_FILTER" ]]; then
    LAUNCH_CMD+=" use_imu_filter:=$LIVOX_FILTER"
  fi

  CMD="(
        source install/setup.bash
        ${TITLE_CMD}
        ${LAUNCH_CMD}
        /bin/bash
       )"

elif [[ "$COMMAND" == "record" ]]; then
  count=0
  OUTPUT=
  FORMAT="mcap"
  TOPICS=
  
  while [[ $count < ${#COMMAND_ARGS[@]} ]]; do
    arg=${COMMAND_ARGS[$count]}

    if [[ "$arg" == "-o" || "$arg" == "--output" ]]; then
      ((count+=1)) && OUTPUT=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "-s" || "$arg" == "--storage" ]]; then
      ((count+=1)) && FORMAT=${COMMAND_ARGS[$count]}
    elif [[ "$arg" == "-t" || "$arg" == "--topics" ]]; then
      ((count+=1)) && arg="${COMMAND_ARGS[$count]}"
      while [[ ! "${arg}" =~ ^[-?] && -n "${arg}" ]]; do
        TOPICS+=$(get_topics_to_record "${arg}")
        ((count+=1)) && arg="${COMMAND_ARGS[$count]}"
      done
      TOPICS="${TOPICS% }"  # remove trailing space
      ((count-=1))  # revisit arg
    elif [[ ! "$arg" =~ ^[-?] ]]; then
      while [[ ! "${arg}" =~ ^[-?] && -n "${arg}" ]]; do
        TOPICS+=$(get_topics_to_record "${arg}")
        ((count+=1)) && arg="${COMMAND_ARGS[$count]}"
      done
      TOPICS="${TOPICS% }"  # remove trailing space
      ((count-=1))  # revisit arg
    else
      printf "$0 record: illegal option -- ${arg}\n"
      exit 1
    fi

    ((count+=1))
  done

  if [[ -z "$TOPICS" ]]; then
    if [[ "$IS_VERBOSE" == "true" && "$IGN_PROMPT" != "true" ]]; then
      read -p "Record all topics ([y]es/[n]o)?  " user_confirm
      echo
    else
      user_confirm="y"
    fi

    if [[ "${user_confirm,,}" == "y" || "${user_confirm,,}" == "yes" ]]; then
      TOPICS+="-a"
    else
      echo "Then specify which topics to record."
      exit 1
    fi
  fi

  RECORD_CMD="ros2 bag record -s ${FORMAT}"
  if [[ -n "$OUTPUT" ]]; then
    RECORD_CMD+=" -o ${OUTPUT}"
  fi
  RECORD_CMD+=" ${TOPICS}"

  CMD="(
        source install/setup.bash
        ${TITLE_CMD}
        ${RECORD_CMD}
        /bin/bash
       )"

else
  # Note: no need to source setup.bash in subshell unless executing ros2
  CMD="(
        ${TITLE_CMD}
        ${COMMAND}
        /bin/bash
       )"
fi

# Display container's default ID and DDS within command subshell ()
if [[ "$IS_VERBOSE" == "true" ]]; then
  ADD="(
        echo
        echo Subshell variables should be consistent with configuration:
        echo ROS2 domain ID is  \$ROS_DOMAIN_ID
        echo RMW/DDS vendor is  \$RMW_IMPLEMENTATION
        echo"
  CMD=${CMD/"("/$ADD}
fi

# Set up display if available
function setup_display() {
  # Try to set DISPLAY if not set
  if [ -z "$DISPLAY" ]; then
      # Check if we're on physical display
      if [ -e /tmp/.X11-unix/X0 ]; then
          export DISPLAY=:0
          echo "Set DISPLAY=:0"
      elif [ -e /tmp/.X11-unix/X1 ]; then
          export DISPLAY=:1
          echo "Set DISPLAY=:1"
      else
          echo "No X11 display found. Running without GUI support."
          return 1
      fi
  fi

  # Test xhost
  if command -v xhost &> /dev/null; then
      xhost +local:docker 2>/dev/null || {
          echo "xhost failed. Running without GUI support."
          return 1
      }
      return 0
  else
      echo "xhost not found. Running without GUI support."
      return 1
  fi
}

#======
# MAIN
#======

WORKSPACE_DIR="/home/num4/wuvin/num4_home/num4_ws"

if [[ "$IS_VERBOSE" == "true" ]]; then
  printf "The folowing commands will be executed in the new terminal:\n"
  printf "$CMD\n"
fi
if [[ "$IS_VERBOSE" == "true" && "$IGN_PROMPT" != "true" ]]; then
  read -p "Do you want to continue ([y]es/[n]o)?  " user_confirm
else
  user_confirm="y"
fi
if [[ "${user_confirm,,}" != "y" && "${user_confirm,,}" != "yes" ]]; then
  echo "User early exit."
  exit 0
fi

# Check if container is already running
if [ $(docker ps -q -f name=${CONTAINER_NAME}) ]; then
  echo "Found container ${CONTAINER_NAME}. "

  if setup_display; then
    echo "Running with GUI support via xhost (DISPLAY=$DISPLAY)"

    docker exec -it "${CONTAINER_NAME}" /bin/bash -c "$CMD"

    xhost -local:docker 2>/dev/null  # cleanup
  else
    echo "Running without GUI support"

    docker exec -it "${CONTAINER_NAME}" /bin/bash -c "$CMD"
  fi

else
  echo "Starting container ${CONTAINER_NAME}..." 

  if setup_display; then
    echo "Running with GUI support via xhost (DISPLAY=$DISPLAY)"

    docker run -it --rm \
               --privileged \
               --name=${CONTAINER_NAME} \
               --network=host \
               --runtime=nvidia \
               -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
               -e RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
               -v /dev:/dev \
               -v /etc/localtime:/etc/localtime:ro \
               -v /etc/timezone:/etc/timezone:ro \
               --device-cgroup-rule "c 81:* rmw" \
               --device-cgroup-rule "c 189:* rmw" \
               -e DISPLAY=$DISPLAY \
               -e QT_X11_NO_MITSHM=1 \
               -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
               -v $HOME/.Xauthority:/root/.Xauthority:ro \
               -v $WORKSPACE_DIR:/num4_ws \
               -v $(pwd):/mnt/pwd \
               --entrypoint /bin/bash \
               ${IMAGE_NAME_TAG} -c "${CMD}"

    xhost -local:docker 2>/dev/null  # cleanup
  else
    echo "Running without GUI support"

    docker run -it --rm \
               --privileged \
               --name=${CONTAINER_NAME} \
               --network=host \
               --runtime=nvidia \
               -v /dev:/dev \
               -v /etc/localtime:/etc/localtime:ro \
               -v /etc/timezone:/etc/timezone:ro \
               --device-cgroup-rule "c 81:* rmw" \
               --device-cgroup-rule "c 189:* rmw" \
               -v $WORKSPACE_DIR:/num4_ws \
               -v $(pwd):/mnt/pwd \
               --entrypoint /bin/bash \
               ${IMAGE_NAME_TAG} -c "${CMD}"
  fi

fi

echo "Container exited."