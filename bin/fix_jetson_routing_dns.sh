#!/bin/bash

# Shell script to run remote-side (i.e., on Jetson) for client to provide
# internet access. Run route_network_interface.sh first on client.
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: December 23, 2025

# Clear any existing default routes
sudo ip route del default 2>/dev/null

# Add laptop as the gateway
sudo ip route add default via 192.168.55.100

# Verify the route
ip route

# Test connectivity to external IP first
echo "Checking connectivity to external IP..."
ping -c 3 8.8.8.8

# If the above works, then fix DNS
if [ $? -eq 0 ]; then
    echo -e "nameserver 8.8.8.8\nnameserver 8.8.4.4" | sudo tee /etc/resolv.conf
else
    echo "Failed connectivity to external IP"
fi

# Force immediate sync
echo "Forcing immediate sync..."
sudo systemctl restart systemd-timesyncd

echo "Checking hardwork clock..."
sudo hwclock
