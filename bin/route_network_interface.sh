#!/bin/bash

# Shell script to run client-side (i.e., on laptop) for forwarding access
# to internet to remote Jetson. Afterwards, run fix_jetson_routing_dns.sh
# on remote Jetson.
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: December 23, 2025

# Define expected address strings for wired and wireless connections
USB_JETSON_STR='192.168.55.0/24'
WLS_JETSON_STR='192.168.0.0/24'

# Determine which device name(s) represents Jetson connection
out=$(ip route)

usbname=$(echo "$out" | grep "$USB_JETSON_STR" | head -n 1 | awk '{print $3}')
wlsname=$(echo "$out" | grep "$WLS_JETSON_STR" | head -n 1 | awk '{print $3}')
if [[ -n ${usbname} ]]; then  # wired Jetson connection
    usbaddr=$(echo "$out" |
        grep "$USB_JETSON_STR" |
        head -n 1 |
        awk '{print $9}')
    echo "Jetson connection (via USB): dev ${usbname} src ${usbaddr}"
    jetname=$usbname
elif [[ -n ${wlsname} ]]; then  # wireless jetson connection
    wlsaddr=$(echo "$out" |
        grep "$WLS_JETSON_STR" |
        head -n 1 |
        awk '{print $9}')
    echo "Jetson connection (via WLS): dev ${wlsname} src ${wlsaddr}"
    jetname=$wlsname
else
    echo "ERROR: Unable to determine connection to Jetson." \
         "Check addresses set in route_network_interface.sh."
    exit 1
fi

# Determine which device name represents internet connection
out=$(ip route get 8.8.8.8 2>/dev/null)

netaddr=$(echo "$out" | awk '/src/ {print $7}')
netname=$(echo "$out" | awk '/dev/ {print $5}')

[[ $netname =~ ^(eth|enp|eno|enx) ]] && iswired="true" || iswired=
[[ -z $iswired && $netname =~ ^(wlan|wlp|wls) ]] && iswired="false"
if [[ $iswired == "true" ]]; then
    echo "Internet connection (wired): dev $netname src $netaddr"
elif [[ $iswired == "false" && $jetname == $usbname ]]; then
    echo "Internet connection (wireless): dev $netname src $netaddr"
elif [[ $iswired == "false" ]]; then
    echo "ERROR: Both internet and Jetson connections are wireless."
    exit 1
else
    echo "ERROR: Unable to interpret name '$netname'."
    exit 1
fi

# Clear previous rules
sudo iptables -t nat -F
sudo iptables -F FORWARD

# Enable IP forwarding
echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward

# Set up NAT
sudo iptables -t nat -A POSTROUTING -o $netname -j MASQUERADE
sudo iptables -A FORWARD -i $jetname -o $netname -j ACCEPT
sudo iptables -A FORWARD -i $netname -o $jetname -m state \
    --state RELATED,ESTABLISHED -j ACCEPT

# Check if rules are applied
sudo iptables -t nat -L -n -v
