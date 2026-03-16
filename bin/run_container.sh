#!/bin/bash

# Script to run ROS2 container with optional GUI support on Jetson
IMAGE_NAME="num4-wuvin:v0"
CONTAINER_NAME="num4_ws"
WORKSPACE_DIR="$(pwd)/num4_ws"

# Function to check and setup display
setup_display() {
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

# Main execution
echo "Starting ROS2 container..."

if setup_display; then
    echo "Running with GUI support via xhost (DISPLAY=$DISPLAY)"

    docker run -it --rm \
        --name $CONTAINER_NAME \
        --runtime nvidia \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$WORKSPACE_DIR":/num4_ws \
        --device /dev/video0 \
        -v /dev:/dev \
        --device-cgroup-rule "c 81:* rmw" \
        --device-cgroup-rule "c 189:* rmw" \
        $IMAGE_NAME

    # Cleanup
    xhost -local:docker 2>/dev/null
else
    echo "Running without GUI support"

    docker run -it --rm \
        --name $CONTAINER_NAME \
        --runtime nvidia \
        --network host \
        --privileged \
        -v "$WORKSPACE_DIR":/num4_ws \
        --device /dev/video0 \
        $IMAGE_NAME
fi

echo "Container exited."
