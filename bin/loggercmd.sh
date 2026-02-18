#!/bin/bash

# Script to log command executions by day. Implement by adding a line in
# /etc/bash.bashrc to source this script. Use `source` so that functions
# defined in this script can be used after execution.
#
# Example additional .bashrc snippet:
#   # Prompt logging of commands
#   FILE=/home/num4/Documents/Logs/cmd/loggercmd.sh
#   if [[ -f "$FILE" ]]; then
#       source "$FILE"
#   fi
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: February 18, 2026

LOG_DIR="/home/num4/Documents/Logs/cmd"

# Function to execute with unsynchronized clock
get_next_file_unsync() {
    mkdir -p "$LOG_DIR/1969"

    local LATEST=$(
        ls "$LOG_DIR/1969/1969-12-31"_*.log 2>/dev/null |
        sort -V |
        tail -n 1
    )
    local BASE="${LATEST%.*}"
    local PREFIX="${BASE%_*}"
    local NUMBER=$((10#${BASE##*_} + 1))

    if [ -z "$LATEST" ]; then
        local NUMBER=0
    else
        local BASE="${LATEST%.*}"
        local NUMBER=$((10#${BASE##*_} + 1))
    fi

    NEXT_FILE=$(printf "%s_%07d.log" "$PREFIX" "$NUMBER")

    echo "$NEXT_FILE"
}

# Function to record command
log_command() {
    # Get last command from history
    local EXIT_STATUS=$?
    local LAST_CMD=$(history 1 | sed 's/^[ ]*[0-9]*[ ]*//')

    # Determine filepath to log file based on date
    local LOG_DATE=$(date +%Y-%m-%d)

    if [ "$LOG_DATE" = "1969-12-31" ]; then
        local LOG_FILE=$(get_next_file_unsync)
    else
        local FILE_DIR="$LOG_DIR/$(date +%Y)/$(date +'%m - %B')"
        local LOG_FILE=$FILE_DIR/"$(date +%Y-%m-%d).log"

        mkdir -p "$FILE_DIR"
    fi

    # Log
    printf "%s [Exit: %3d] %s\n" \
        "$(date '+%H:%M:%S')" \
        "$EXIT_STATUS" \
        "$LAST_CMD" >> "$LOG_FILE"
}

# Run function after every command
export PROMPT_COMMAND="log_command; $PROMT_COMMAND"