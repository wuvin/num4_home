#!/bin/bash

# Logs when the Jetson is powered on and when there is an SSH connection.
# Also logs when shutdown command is executed. 
#
# On SSH logins, shows a clock check and waits for NTP sync if the clock
# is unsynchronized and a default gateway exists. This can be skipped by
# hitting Return; the keypress is consumed here and avoids showing up in
# the shell's log or its history.
#
# Each logged event retains the first 8 characters of the kernel boot ID
# currently stored. This ID is also indicated by loggercmd.sh within the
# UNSYNCED SESSION separator lines, so entries in both logs can be found
# and matched together to the same boot.
#
# To use this, place loggerin.service in /etc/systemd/system, and modify
# /etc/bash.bashrc to execute this script for interactive terminals with
# SSH logins as well. Note that both files should be made executable.
#
# Example additional .bashrc snippet:
#   # Record login events from SSH
#   FILE=/home/num4/Documents/Logs/loggerin.sh
#   if [[ -f "$FILE" && -x "$FILE" && -n "$SSH_CONNECTION" &&
#         -z "$LOGGERIN_DONE" ]]; then
#       export LOGGERIN_DONE=1
#       "$FILE" "SSH_FROM_${SSH_CLIENT%% *}"
#   fi
#   unset FILE
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: October 21, 2026

LOG_FILE="/home/num4/Documents/Logs/login.log"
LOG_EVENT=${1:-TERMINAL}
MAX_WAIT=60  # seconds to wait for NTP synchronization

NTP_MARKER=/run/systemd/timesync/synchronized   # created by timesyncd
TIMESYNC_CLOCK=/var/lib/systemd/timesync/clock  # touched by timesyncd
DATE_FMT='%a %b %e %H:%M:%S %Y'                 # same style as sshd
BOOT_ID=$(cut -c1-8 /proc/sys/kernel/random/boot_id 2>/dev/null)

CLIENT_IP=${SSH_CLIENT%% *}

# Only talk to user for SSH logins attached to terminal
INTERACTIVE=false
if [[ -n "$SSH_CONNECTION" && -t 0 && -t 1 ]]; then
    INTERACTIVE=true
fi

# Print clock state: ntp or unsynced
clock_source() {
    if [ -e "$NTP_MARKER" ]; then
        echo "ntp"
    else
        echo "unsynced"
    fi
}

clock_desc() {
    case $(clock_source) in
        ntp)    echo "synchronized (NTP)" ;;
        *)      echo "NOT synchronized" ;;
    esac
}

fmt_epoch() {
    date -d "@$1" +"$DATE_FMT"
}

# Check if default gateway exists
HAS_GATEWAY=false
if [ "$LOG_EVENT" != "SHUTDOWN" ]; then
    for ii in {1..2}; do  # twice in case handshake is about to happen
        if ip route show default 2>/dev/null | grep -q "default"; then
            HAS_GATEWAY=true
            break
        fi
        (( ii < 2 )) && sleep 1
    done
fi

# Show clock check
if $INTERACTIVE; then
    NOW=$(date +%s)
    echo  # \n
    echo "Clock check"
    printf '  %-11s %s  [%s]\n' "Now" "$(fmt_epoch "$NOW")" "$(clock_desc)"

    # Last line of event log with timestamp reformatted
    LAST_LINE=$(tail -n 1 "$LOG_FILE" 2>/dev/null)
    if [ -n "$LAST_LINE" ]; then
        LAST_EPOCH=$(date -d "${LAST_LINE:0:19}" +%s 2>/dev/null)
        if [ -n "$LAST_EPOCH" ]; then
            LAST_REST=${LAST_LINE:20}
            printf '  %-11s %s\n' "Last event" "$(fmt_epoch "$LAST_EPOCH")"
            printf '  %-11s %s\n' "" "${LAST_REST#- }"
        else
            printf '  %-11s %s\n' "Last event" "$LAST_LINE"
        fi
    fi

    # Last successful NTP sync as recorded by timesyncd
    SYNC_EPOCH=$(stat -c %Y "$TIMESYNC_CLOCK" 2>/dev/null)
    if [ -n "$SYNC_EPOCH" ]; then
        printf '  %-11s %s\n' "Last sync" "$(fmt_epoch "$SYNC_EPOCH")"
    else
        printf '  %-11s %s\n' "Last sync" "none recorded"
    fi

    printf '  %-11s %s\n' "Gateway" \
        "$($HAS_GATEWAY && echo "found" || echo "none")"
    
    # Flag a clock that has gone backwards since last logged event
    if [ -n "$LAST_EPOCH" ] && (( NOW < LAST_EPOCH )); then
        echo "  ! Clock is earlier than last logged event."
    fi
fi

# Wait for clock synchronization if route to internet is available
WAITED=0
SKIPPED=false
if [[ "$(clock_source)" = "unsynced" &&
      ( "$HAS_GATEWAY" = true || "$LOG_EVENT" = "POWER_ON" ) ]]; then
    if $INTERACTIVE; then
        echo  # \n
        echo "Hit return to continue without clock synchronization."
    fi

    while [ "$(clock_source)" = "unsynced" ] && (( WAITED < MAX_WAIT )); do
        if $INTERACTIVE; then
            read -r -s -t 1 _
            RC=$?
            if (( RC == 0 )); then  # press return
                SKIPPED=true
                break
            elif (( RC <= 128 )); then  # not a timeout (e.g., EOF)
                sleep 1
            fi
        else
            sleep 1
        fi

        WAITED=$((WAITED + 1))
        if $INTERACTIVE && (( WAITED % 10 == 0 )); then
            echo "Awaiting clock synchronization... ($WAITED/$MAX_WAIT)"
        fi
    done

    if $INTERACTIVE; then
        if $SKIPPED; then
            echo "Skipped clock synchronization after $WAITED s."
        elif [ "$(clock_source)" = "unsynced" ]; then
            echo "Clock synchronization timed out."
        else
            echo "Clock synchronized after $WAITED s."
        fi
    fi
fi

# Indicate current login
if $INTERACTIVE; then
    echo  # \n
    echo "Current login: $(date +"$DATE_FMT") from $CLIENT_IP"
    echo "Clock: $(clock_desc)"
fi

# Log event
EVENT_STR="- Event: $LOG_EVENT - Boot: ${BOOT_ID:-unknown}"
EVENT_STR+=" - Clock: $(clock_source)"
if [[ "$LOG_EVENT" != "SHUTDOWN" ]]; then
    EVENT_STR+=" - Gateway: $($HAS_GATEWAY && echo yes || echo no)"
    EVENT_STR+=" - Waited: ${WAITED} s"
    $SKIPPED && EVENT_STR+=" (skipped)"
fi

mkdir -p "$(dirname "$LOG_FILE")"
echo "$(date '+%Y-%m-%d %H:%M:%S') $EVENT_STR" >> "$LOG_FILE"