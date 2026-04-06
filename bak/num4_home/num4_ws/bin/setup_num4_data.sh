#!/bin/bash

read -p "Enter num4 address:  " num4_addr
read -p "Enter password to num4:  " num4_pass

gnome-terminal \
    --tab -- bash -c "sshpass -p \"$num4_pass\" ssh -Yt num4@$num4_addr 'cd wuvin/num4_home && bash num4_ws/bin/run_num4_container.sh -vy --title \"Container Main\"; bash'; exec bash"
    # --tab -- bash -c "./run_num4_container.sh -vy --title \"Foxbridge\" foxbridge; exec bash" \
    # --tab --title="RealSense" -- bash -c "./run_num4_container.sh -vy realsense; exec bash" \
    # --tab -- bash -c "./run_num4_container.sh -vy --title Livox livox; exec bash"