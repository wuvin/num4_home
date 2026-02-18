#!/bin/bash

# Configure which tabs to open (see run_num4_container.sh)
  OPEN_INIT=true  # initialize container
OPEN_BRIDGE=true  # run Foxglove bridge
 OPEN_LIVOX=true  # run Livox driver
    OPEN_RS=true  # run RealSense driver
OPEN_PC2MSG=true  # node_pc2msg: convert Custom msg to PointCloud2
OPEN_TFPUBL=true  # node_tfpubl: publish camera-livox transform
OPEN_PC2IMG=true  # node_pc2img: project point clouds onto image plane
# OPEN_RECORD=true  # TBD: for now, just attach to container as needed

# Configure SSH into Jetson
DEFAULT_NAME=num4@192.168.55.1
DEFAULT_PASS=num4

read -p "Enter name (default: $DEFAULT_NAME):  " NUM4_NAME

if [[ -z $NUM4_NAME ]]; then
    NUM4_NAME=$DEFAULT_NAME
    NUM4_PASS=$DEFAULT_PASS
else
    read -p "Enter password:  " NUM4_PASS
fi

SSH_CMD="sshpass -p $NUM4_PASS ssh -Yt $NUM4_NAME"

# Define strings to be passed into new terminal as part of command
VAR=\\\$  # need 2 lvls to pass: \\\$ -> bash -c "\$" -> $ in subshell
ESC=\\\\  # need 2 lvls to pass: \\\\ -> bash -c "\\" -> \ in subshell

# Terminal: initialize container
WORKSPACE_DIR="~/wuvin/num4_home"
BASH_FILEPATH="$WORKSPACE_DIR/num4_ws/bin/run_num4_container.sh"

INIT_CMD="(
           echo $NUM4_PASS | sudo -S echo 'Just a moment...'
           cd $WORKSPACE_DIR
           bash $BASH_FILEPATH -vy --title 'Container Main' $ESC
           'touch /mnt/pwd/init_done'
           rm -f init_done
          )"  # note: without bash, terminal closes after container exit

if [[ $OPEN_INIT == "true" ]]; then
    echo "Terminal: container initialization"

    gnome-terminal --tab -- bash -c "$SSH_CMD \"$INIT_CMD\""

    # Wait for init_done to ensure this terminal is container main
    CHECK_INIT="$SSH_CMD -q test -f $WORKSPACE_DIR/init_done"
    echo "Waiting for container to create init_done before continuing..."

    sleep 1
    $CHECK_INIT
    while [ $? -eq 1 ]; do
        echo "Still waiting..."
        sleep 1
        $CHECK_INIT
    done

    echo "Initialization done."
fi

# Terminal: run bridge for Foxglove
BRIDGE_CMD="(
             cd $WORKSPACE_DIR
             bash $BASH_FILEPATH -vy --title 'Foxbridge' foxbridge
            )"  # note: terminal closes after container exit

if [[ $OPEN_BRIDGE == "true" ]]; then
    echo "Terminal: Foxbridge"

    gnome-terminal --tab -- bash -c "$SSH_CMD \"$BRIDGE_CMD\""
fi

# Terminal: set up Livox LiDAR
LIVOX_CMD="(
            cd $WORKSPACE_DIR
            bash $BASH_FILEPATH -vy --title 'Livox' livox
           )"  # note: terminal closes after container exit

if [[ $OPEN_LIVOX == "true" ]]; then
    echo "Terminal: Livox"

    gnome-terminal --tab -- bash -c "$SSH_CMD \"$LIVOX_CMD\""
fi

# Terminal: set up RealSense
RS_CMD="(
         cd $WORKSPACE_DIR
         bash $BASH_FILEPATH -vy --title 'RealSense' realsense
        )"  # note: terminal closes after container exit

if [[ $OPEN_RS == "true" ]]; then
    echo "Terminal: RealSense"

    gnome-terminal --tab -- bash -c "$SSH_CMD \"$RS_CMD\""
fi

# Terminal: convert Custom messages into PointCloud2
PC2MSG_CMD="(
             cd $WORKSPACE_DIR
             bash $BASH_FILEPATH -vy --title 'PC2MSG' $ESC
             'source install/setup.bash && $ESC
             ros2 run livox_to_pointcloud2 livox_to_pointcloud2_node'
            )"

if [[ $OPEN_PC2MSG == "true" ]]; then
    echo "Terminal: PC2MSG"

    gnome-terminal --tab -- bash -c "$SSH_CMD \"$PC2MSG_CMD\""
fi

# Terminal: publish camera-to-Livox transformation
TFPUBL_CMD="(
             cd $WORKSPACE_DIR
             bash $BASH_FILEPATH -vy --title 'TFPUBL' $ESC
             'source install/setup.bash && $ESC
             ros2 run tf2_ros static_transform_publisher $ESC
             0 0 0 $ESC
             -1.5708 -1.5708 0 $ESC
             camera_color_optical_frame livox_frame'
            )"

if [[ $OPEN_TFPUBL == "true" ]]; then
    echo "Terminal: TFPUBL"

    gnome-terminal --tab -- bash -c "$SSH_CMD \"$TFPUBL_CMD\""
fi

# Terminal: project point clouds onto image plane
PC2IMG_CMD="(
             cd $WORKSPACE_DIR
             bash $BASH_FILEPATH -vy --title 'PC2IMG' $ESC
             'source install/setup.bash && $ESC
             ros2 launch pc2image launch_projector.py'
            )"

if [[ $OPEN_PC2IMG == "true" ]]; then
    echo "Terminal: PC2IMG"

    gnome-terminal --tab -- bash -c "$SSH_CMD \"$PC2IMG_CMD\""
fi

exit 0

# Terminal: run recording
# TBD: for now, just attach to container and record as needed