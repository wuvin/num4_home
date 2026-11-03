#!/bin/bash

# Shell script to execute on remote to run multiple ROS 2 nodes within a
# tmux session. A common use case is setting up the sensor & SLAM stack.
#
# Note: can also execute client-side interactive via `ssh -t num4@<addr>
# "/path/to/file.sh"`.
#
# Usage:
#   setup_num4_tmux.sh start    start default node stack or reattach
#   setup_num4_tmux.sh attach   attach or switch client
#   setup_num4_tmux.sh stop     Ctrl-C every node and end session
#                               (stops bag recordings first)
#                               (use --keep to keep container running)
#
# The tmux session is called `num4`.
#
# tmux tabs:
#   host        first tab
#   init        container launcher
#   mavros      ROS 2 and MAVLink
#   sensors     camera | lidar | static TF (split panes)
#   slam        FAST-LIVO2 perception
#   other       typically for bag recording
#
# Do not forget to `chmod +x setup_num4_tmux.sh`.
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: November 02, 2026

# Set script to crash instantly on some glitches for easier debugging
set -euo pipefail  # unexpected errs, misspelled vars, failing pipelines

# Configure session
SESSION="num4"
CONTAINER="num4_ros2_humble"
CONTAINER_TIMEOUT=60    # seconds to wait for container to come up
CONTAINER_SETTLE=2      # seconds to let new container shells settle
PANE_TIMEOUT=10         # seconds for pane to wait for docker
NODE_KILL_WAIT=5        # seconds to wait for lengthy nodes to die
CONTAINER_EXIT_WAIT=10  # seconds to wait for container to exit
SLAM_DELAY=10           # seconds after sensor start before running SLAM

# Configure pathing
RUN_SH="/home/num4/m4-autonomy/m4-docker/run-docker-addtl.sh"
BUILD_DIR="/home/num4/m4-autonomy/m4_home/"

# Configure executable commands (Jetson)
INIT_CMD="${RUN_SH} ${BUILD_DIR}"
ATTACH_CMD="docker exec -it -e DISPLAY ${CONTAINER} bash"
    # -e DISPLAY passes shell's X11-forwarded display into container;
    # matters if container was started in earlier SSH session

# Configure executable commands (container)
MAVROS_CMD="ros2 launch m4_firmware interfaces.launch.py \
    fcu_url:=/dev/ttyTHS1:921600"
CAMERA_CMD="ros2 launch realsense2_bringup d455.launch.py \
    rs_config:=num4.yaml"
LIDAR_CMD="ros2 launch livox_bringup mid360.launch.py \
    config:=num4/config.json"
TFPUBL_CMD="ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 2.61799 \
    --frame-id camera_link --child-frame-id livox_frame"
SLAM_CMD="ros2 launch fastlivo2_bringup mapping_num4.launch.py \
    camera:=D455 lidar:=MID360"

# Configure command to display/type into other window (usually bag)
SHOW_OTHER_CMD='/home/num4/record_num4_bag.sh default'

# Define helper functions (container status)
is_container_running() {
    local status=$(
        docker inspect \
        -f '{{.State.Running}}' \
        "$CONTAINER" \
        2>/dev/null
    )

    [[ "${status}" == "true" ]]  # return
}
wait_for_container() {  # idle until timeout or container detected
    local ctr=0
    printf "Waiting for container ${CONTAINER}"
    until is_container_running; do
        if (( ctr >= CONTAINER_TIMEOUT )); then
            printf "\nFailed to start container."
            printf " Check init window: tmux attach -t ${SESSION}\n"
            exit 1
        fi
        sleep 1 && ctr=$((ctr+1)) && printf '.'
    done
    printf " up.\n"
}

# Define helper functions (tmux session)
new_window() {  # $1=name
    tmux new-window \
        -t "${SESSION}:" \
        -n "$1" \
        -c "${HOME}" \
        -P \
        -F '#{pane_id}'
}
split_window() {  # $1=tab(1), $2=tab(2)
    tmux split-window \
        -t "$1" "$2" \
        -c "${HOME}" \
        -P \
        -F '#{pane_id}'
}
label() {  # $1=tab, $2=label
    tmux set-option \
        -p \
        -t "$1" @label "$2"
}
type_cmd() {  # $1=tab, $2=cmd
    tmux send-keys \
        -t "$1" \
        -l "$2"
}
exec_cmd() {  # $1=tab, $2=cmd
    type_cmd "$1" "$2"
    tmux send-keys \
        -t "$1" \
        Enter
}
attach() {
    if [[ -n "${TMUX:-}" ]]; then
        printf "Switching client."
        exec tmux switch-client -t "${SESSION}"
    else
        printf "Attaching."
        exec tmux attach -t "${SESSION}"
    fi
}

# Define helper functions (tmux status; docker in pane's foreground)
is_pane_ready() {  # $1=tab/pane
    local status=$(
        tmux display-message \
        -p \
        -t "$1" \
        '#{pane_current_command}'
    )

    [[ "${status}" == docker ]]  # return
}
wait_for_pane() {  # $1=tab/pane
    local ctr=0
    until is_pane_ready "$1"; do
        if (( ctr >= PANE_TIMEOUT )); then
            printf "\nNew window failed to attach to container."
            exit 1
        fi
        sleep 1 && ctr=$((ctr+1)) && printf '.'
    done
    sleep 1  # let bash inside container reach prompt
}

# Start up tmux session
start() {
    # Initial checks
    command -v tmux >/dev/null || {  # check if Jetson has tmux
        echo "tmux is not installed."
        exit 1
    }

    if tmux has-session -t "=${SESSION}" 2>/dev/null; then
        echo "Session '${SESSION}' already running; reattaching."
        attach
    fi

    # Host session
    local host
    host=$(
        tmux new-session \
        -d \
        -s "${SESSION}" \
        -n host \
        -c "${HOME}" \
        -P \
        -F '#{pane_id}'
    )

    tmux set-option -t "${SESSION}" mouse on  # click tabs/panes, scroll
    tmux set-option -t "${SESSION}" history-limit 50000  # log length

    # Init tab
    local init mavros camera lidar tfpubl slam other

    init=$(new_window init)
    if is_container_running; then
        echo "Found container ${CONTAINER}. Attaching."
        exec_cmd "${init}" "${ATTACH_CMD}"
        wait_for_pane "${init}"
    else
        echo "Initializing new container ${CONTAINER}."
        exec_cmd "${init}" "${INIT_CMD}"
        tmux set-option -t "${SESSION}" @owns_container 1  # init = main
        wait_for_container
        sleep "${CONTAINER_SETTLE}"
    fi
    
    exec_cmd "${init}" 'echo -e "\nDetails:"'
    exec_cmd "${init}" "echo -e '\tContainer Name      ${CONTAINER}'"
    exec_cmd "${init}" 'echo -e "\tROS_DOMAIN_ID       ${ROS_DOMAIN_ID}"'
    exec_cmd "${init}" 'echo -e "\tRMW_IMPLEMENTATION  ${RMW_IMPLEMENTATION}"'

    # MAVROS
    mavros=$(new_window mavros)
    exec_cmd "${mavros}" "${ATTACH_CMD}"
    wait_for_pane "${mavros}"
    exec_cmd "${mavros}" "${MAVROS_CMD}"

    # Sensors (camera, lidar, tfpubl)
    camera=$(new_window sensors)
    lidar=$(split_window "${camera}" -h)
    tfpubl=$(split_window "${lidar}" -v)

    tmux set-option -w -t "${camera}" pane-border-status top
    tmux set-option -w -t "${camera}" pane-border-format ' #{@label} '

    for pane in camera lidar tfpubl; do
        label "${!pane}" "$pane"
        exec_cmd "${!pane}" "${ATTACH_CMD}"
        wait_for_pane "${!pane}"
        cmd="${pane^^}_CMD"
        exec_cmd "${!pane}" "${!cmd}"
    done

    # SLAM
    slam=$(new_window slam)

    exec_cmd "${slam}" "${ATTACH_CMD}"
    wait_for_pane "${slam}"
    exec_cmd "${slam}" \
        "echo 'Delaying SLAM by ${SLAM_DELAY} seconds for sensors...'"
    exec_cmd "${slam}" "sleep ${SLAM_DELAY} && ${SLAM_CMD}"

    # Other
    other=$(new_window other)

    exec_cmd "${other}" "${ATTACH_CMD}"
    wait_for_pane "${other}"
    type_cmd "${other}" "${SHOW_OTHER_CMD}"

    # Active window
    tmux select-window -t "${SESSION}:sensors"
    attach
}

# Stop container
stop_container() {
    is_container_running || return 0

    # Ctrl-D main shell
    local is_main=$(
        tmux show-options \
        -t "${SESSION}" \
        -qv @owns_container
    )

    if [[ "${is_main}" == 1 ]]; then
        printf "Exiting main container shell in init tab..."
        exec_cmd "${SESSION}:init" "exit"
        local ctr=0
        while is_container_running && (( ctr < CONTAINER_EXIT_WAIT )); do
            sleep 1 && ctr=$((ctr+1)) && printf "."
        done
        is_container_running && printf " timed out.\n" || printf " done.\n"
    fi

    # Ctrl-D fallback if container started elsewhere or did not exit
    if is_container_running; then
        printf "Running docker stop on ${CONTAINER}..."
        docker stop -t 5 "${CONTAINER}" >/dev/null \
            && printf " done.\n" \
            || printf " docker stop failed; check 'docker ps'.\n"
    fi
    
    is_container_running || printf "Container ${CONTAINER} stopped.\n"
}

# Stop tmux session
stop () {
    if ! tmux has-session -t "=${SESSION}" 2>/dev/null; then
        echo "No tmux session '${SESSION}' found."
        return
    fi

    echo "Stopping bag recording (if any)..."
    tmux send-keys -t "${SESSION}:other" C-c 2>/dev/null || true
    sleep "${NODE_KILL_WAIT}"

    echo "Sending Ctrl-C to every node..."
    for pane in $(tmux list-panes -s -t "${SESSION}" -F '#{pane_id}'); do
        tmux send-keys -t "$pane" C-c
    done
    sleep "${NODE_KILL_WAIT}"

    if [[ "${1:-}" == "--keep" ]]; then
        echo "Leaving container ${CONTAINER} running."
    else
        stop_container
    fi

    echo "Session '${SESSION}' closing..."
    tmux kill-session -t "${SESSION}"
}

# Main
case "${1:-start}" in
    start)  start ;;
    attach) attach ;;
    stop)   stop "${2:-}" ;;
    *)      echo "Usage: $0 [start|attach|stop [--keep]]"; exit 1 ;;
esac