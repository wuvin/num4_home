#!/bin/bash

# Script for DYNAMIXEL Wizard 2.0 to access U2D2 connected to some remote
# Jetson. Performs SSH port forwarding to accomplish this.
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: December 23, 2025

DEFAULT_JET_ADDR=192.168.55.1
DEFAULT_USB_PORT=/dev/ttyUSB0
DEFAULT_OUT_PORT=/tmp/ttyJetsonUSB

read -e -p "Enter 0 for Jetson, 1 for laptop:  " -i 1 is_laptop

if [[ $is_laptop == '0' ]]; then  # on Jetson
    read -e -p "Enter USB port:  " -i "$DEFAULT_USB_PORT" usb_port
    socat TCP-LISTEN:8888,reuseaddr,fork FILE:$usb_port,b57600,raw
elif [[ $is_laptop == '1' ]]; then  # on laptop
    read -e -p "Enter Jetson IP address:  " -i "$DEFAULT_JET_ADDR" jet_addr
    read -e -p "Enter port to forward:  " -i "$DEFAULT_OUT_PORT" out_port
    socat PTY,link=$out_port,raw TCP:$jet_addr:8888
else
    echo "Invalid input $is_laptop"
    exit 0
fi
