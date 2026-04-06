#!/bin/bash

# Preloaded bash commands
PRERUN="${1:-false}"
if [[ "$PRERUN" == "true"* ]]; then
  echo "Pre-loading bash commands to execute for terminal..."
  TMP="roslaunch m4_base m4_control.launch"
  TMP="When ready to morph NUM4 into drive, execute:  ${TMP}"
  CMD="(
        echo hi
        cd m4_ws/src/m4_base
        export TEST_VAR=69
        tmux new -s mavros -d
        tmux send-keys -t mavros \"source ../../devel/setup.bash\" C-m
        tmux send-keys -t mavros \"roslaunch m4_base interfaces.launch\" C-m
        tmux new -s controls -d
        tmux send-keys -t controls \"source ../../devel/setup.bash\" C-m
        tmux send-keys -t controls \"echo '${TMP}'\" C-m
        tmux at -t controls
        /bin/bash
       )"
  unset TMP
else
  CMD="/bin/bash"
fi

echo "/bin/bash -c ${CMD}"

# Name of the Docker image
IMAGE_NAME="m4:ROS-Noetic"

# Name of the Docker container
CONTAINER_NAME="num4_teleop"

# Path to the directory you want to mount inside the container
HOST_DIRECTORY_PATH="/home/num4/m4-autonomy/m4-firmware/m4_home"
CONTAINER_DIRECTORY_PATH="/home/m4"

# Check if the container is already running
echo "Running container ${CONTAINER_NAME} from image ${IMAGE_NAME}..."
if [ $(docker ps -q -f name=${CONTAINER_NAME}) ]; then
  echo "Container ${CONTAINER_NAME} is already running."
else
  # Run the container
  docker run -it --rm --privileged --name ${CONTAINER_NAME} \
             --network="host" \
             -v ${HOST_DIRECTORY_PATH}:${CONTAINER_DIRECTORY_PATH} \
             -v /etc/timezone:/etc/timezone:ro -v /etc/localtime:/etc/localtime:ro \
	     -v /dev:/dev \
	     --device-cgroup-rule "c 81:* rmw" \
	     --device-cgroup-rule "c 189:* rmw" \
             ${IMAGE_NAME} /bin/bash -c "${CMD}"
fi
