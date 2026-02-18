#!/bin/bash

# Logs when the Jetson is powered on and when there is an SSH connection.
# Also logs when shutdown command is executed. 
#
# To use this, place loggerin.service in /etc/systemd/system, and modify
# /etc/bash.bashrc to execute this script for interactive terminals with
# SSH logins as well. Note that both files should be made executable.
#
# Example additional .bashrc snippet:
#   # Record login events from SSH
#   FILE=/home/num4/Documents/Logs/loggerin.sh
#   if [[ -f "$FILE" && -x "$FILE" && -n "$SSH_CONNECTION" ]]; then
#       "$FILE" "SSH_FROM_${SSH_CLIENT%% *}"
#   fi
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: February 18, 2026

LOG_FILE="/home/num4/Documents/Logs/login.log"
LOG_EVENT=${1:-TERMINAL}
COUNTER=0
MAX_NITR=60
MIN_YEAR=2025

# Check if default gateway exists
IS_OFFLINE=true
for ii in {1..2}; do  # twice in case handshake is about to happen
    if ip route show default | grep -q "default"; then
        IS_OFFLINE=false
        break
    fi

    sleep 1
done

# Indicate to user if route to internet is available
if [[ -n "$SSH_CONNECTION" && "$IS_OFFLINE" = false ]]; then
    echo -e "\nDefault gateway found."
elif [[ -n "$SSH_CONNECTION" && "$IS_OFFLINE" = true ]]; then  # verbose
    echo -e "\nNo default gateway found."
fi

# Wait for clock synchronization if route to internet is available
if [[ "$IS_OFFLINE" = false || "$LOG_EVENT" = "POWER_ON" ]]; then
    while [ $(date +%Y) -lt $MIN_YEAR ]; do
        COUNTER=$((COUNTER+1))

        if [ $COUNTER -ge $MAX_NITR ]; then
            break
        fi

        if [[ -n "$SSH_CONNECTION" ]] && (( COUNTER % 10 == 0 )); then
            echo "Awaiting clock synchronization... ($COUNTER/$MAX_NITR)"
        fi

        sleep 1
    done
fi

# Indicate current login
if [ -n "$SSH_CONNECTION" ]; then
    SSH_STR="from ${SSH_CLIENT%% *}"
    echo "Current login: $(date +'%a %b %d %H:%M:%S %Y') $SSH_STR"
fi

# Log event
mkdir -p "$(dirname $LOG_FILE)"
if [[ "$LOG_EVENT" = "POWER_ON" || "$LOG_EVENT" = "SHUTDOWN" ]]; then
    EVENT_STR="- Event: $LOG_EVENT"
elif [ "$IS_OFFLINE" = true ]; then
    EVENT_STR="- Event: $LOG_EVENT - NO DEFAULT GATEWAY FOUND"
else
    EVENT_STR="- Event: $LOG_EVENT - Attempts: $COUNTER"
fi

echo "$(date '+%Y-%m-%d %H:%M:%S') $EVENT_STR" >> $LOG_FILE