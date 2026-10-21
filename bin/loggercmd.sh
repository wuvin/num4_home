#!/bin/bash

# Script to log command executions by day. Implement by adding a line in
# /etc/bash.bashrc to source this script. Use `source` so that functions
# defined in this script can be used after execution.
#
# A note on `HISTCONTROL` environment variable: logging each command via
# `history 1` relies on the setting of `HISTCONTROL`, which, IIRC, is by
# default set to "ignoreboth" in ~/.bashrc. The desired functionality in
# this logger is to instead log exactly what was executed. This includes
# consecutive repeats and commands typed with a leading space, so change
# to `HISTCONTROL` may be needed.
#
# Hitting Return on an empty line and the first prompt of a shell should
# log nothing. See logcmd_sync_histcontrol() for how this works, without
# changing what is saved to ~/.bash_history.
#
# Logs commands from num4 and root shells into num4's log folders; lines
# belong to root are tagged [root], while the files stay owend by num4.
#
# When the clock has not been synchronized since boot, commands are sent
# and appended to the most recent dated log file with timestamps elapsed
# since boot (counting up from 00:00:00). These appear after a separator
# line that includes the boot ID. The resulting break in time marks when
# there is an unsynchronized session which happened sometime between the
# file's date and the next dated file.
#
# Example additional .bashrc snippet:
#   # Prompt logging of commands
#   FILE=/home/num4/Documents/Logs/cmd/loggercmd.sh
#   if [[ -f "$FILE" ]]; then
#       source "$FILE"
#   fi
#   unset FILE
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: October 21, 2026

LOG_OWNER="num4"
LOG_DIR="/home/num4/Documents/Logs/cmd"
NTP_MARKER=/run/systemd/timesync/synchronized  # created by timesyncd
NO_DATED_LOG="$LOG_DIR/1969/1969-12-31.log"    # used when no dated file

# Only log for num4 and root
LOGCMD_USER=$(id -un)
if [[ $EUID -ne 0 && "$LOGCMD_USER" != "$LOG_OWNER" ]]; then
    return 0
fi

# Print path to latest dated log file, YYYY/MM - Month/YYYY-MM-DD.log
latest_dated_log() {
    local FILES=(
        "$LOG_DIR"/20[0-9][0-9]/*/20[0-9][0-9]-[0-9][0-9]-[0-9][0-9].log
    )
    local LATEST=$(
        printf '%s\n' "${FILES[@]}" | LC_ALL=C sort | tail -n 1
    )
    [ -f "$LATEST" ] && echo "$LATEST"
}

# Print time since boot as HH:MM:SS (hr may exceed 24)
uptime_hms() {
    local UP
    read -r UP _ < /proc/uptime
    UP=${UP%.*}
    local HH=$((UP / 3600))
    local MM=$((UP % 3600 / 60))
    local SS=$((UP % 60))
    printf '%02d:%02d:%02d' $HH $MM $SS
}

# Give files and folders back to log owner when running as root
fix_owner() {
    if (( EUID == 0 )); then
        chown "$LOG_OWNER:$LOG_OWNER" "$@" 2>/dev/null
    fi
    return 0
}

# Choose log file for unsynchronized session, then mark start
#   Note: runs in current shell, not a subshell, so UNSYNC_LOG_FILE
#   persists
start_unsync_session() {
    local FILE=$(latest_dated_log)
    if [ -z "$FILE" ]; then
        FILE=$NO_DATED_LOG
        mkdir -p "$(dirname "$FILE")"
        fix_owner "$(dirname "$FILE")"
    fi

    local BOOT_ID=$(cut -c1-8 /proc/sys/kernel/random/boot_id 2>/dev/null)
    printf -- \
        '--- UNSYNCED SESSION boot=%s (times below are since boot) ---\n' \
        "${BOOT_ID:-unknown}" >> "$FILE"
    fix_owner "$FILE"

    UNSYNC_LOG_FILE=$FILE
}

logcmd_sync_histcontrol() {
    if [[ "${HISTCONTROL-}" != "${LOGCMD_LIVE_HC-unset}" ]]; then
        LOGCMD_USER_HC=${HISTCONTROL-}

        local IFS=: TOKEN KEEP=()
        for TOKEN in $LOGCMD_USER_HC; do
            case $TOKEN in
                ignorespace|ignoredups|ignoreboth) ;;
                *) KEEP+=("$TOKEN") ;;
            esac
        done

        HISTCONTROL="${KEEP[*]}"
        LOGCMD_LIVE_HC=$HISTCONTROL
    fi
}

# Function to record command
log_command() {
    local EXIT_STATUS=$?  # must come first, b4 any other cmd
    local HIST HNUM RAW CMD DROP=false
    local LOG_FILE TIMESTAMP YEAR MONTH_DIR DAY TAG=""

    logcmd_sync_histcontrol

    # Get newest history entry, printed as "<number><* or space> <cmd>"
    HIST=$(HISTTIMEFORMAT= builtin history 1)
    HIST=${HIST#"${HIST%%[![:space:]]*}"}  # strip leading whitespace
    HNUM=${HIST%%[!0-9]*}
    RAW=${HIST:${#HNUM}+2}  # cmd as typed

    # Avoid logging previous session's last cmd as first cmd
    if [ -z "${LOGCMD_LAST_HNUM+set}" ]; then
        LOGCMD_LAST_HNUM=$HNUM
        LOGCMD_PREV_KEPT=$RAW
        return
    fi

    # Skip when nothing was executed (e.g., hitting Return on empty line)
    [ "$HNUM" = "$LOGCMD_LAST_HNUM" ] && return

    # Delete entry from history if user's HISTCONTROL excludes it
    case ":$LOGCMD_USER_HC:" in
        *:ignorespace:*|*:ignoreboth:*)
            [[ "$RAW" == [[:space:]]* ]] && DROP=true ;;
    esac
    case ":$LOGCMD_USER_HC:" in
        *:ignoredups:*|*:ignoreboth:*)
            [[ "$RAW" == "$LOGCMD_PREV_KEPT" ]] && DROP=true ;;
    esac
    if $DROP; then
        builtin history -d "$HNUM"
        LOGCMD_LAST_HNUM=$((HNUM - 1))  # next entry reuses this number
    else
        LOGCMD_LAST_HNUM=$HNUM
        LOGCMD_PREV_KEPT=$RAW
    fi

    CMD=${RAW#"${RAW%%[![:space:]]*}"}  # drop leading whitespace for log
    (( EUID == 0 )) && TAG="[root] "

    # Check if clock has been synchronized
    #   Note: instead of checking for a 1969 date (as initially done),
    #   rely on marker files in /run, which clear on every reboot, as
    #   timesyncd creates marker on first successful sync
    if [ ! -e "$NTP_MARKER" ]; then
        [ -n "$UNSYNC_LOG_FILE" ] || start_unsync_session
        LOG_FILE=$UNSYNC_LOG_FILE
        TIMESTAMP=$(uptime_hms)
    else
        # One date call so values avoid straddling midnight
        IFS='|' read -r YEAR MONTH_DIR DAY TIMESTAMP \
            <<< "$(date '+%Y|%m - %B|%Y-%m-%d|%H:%M:%S')"
        mkdir -p "$LOG_DIR/$YEAR/$MONTH_DIR"
        LOG_FILE="$LOG_DIR/$YEAR/$MONTH_DIR/$DAY.log"
    fi

    # Log
    printf "%s [Exit: %3d] %s%s\n" \
        "$TIMESTAMP" \
        "$EXIT_STATUS" \
        "$TAG" \
        "$CMD" >> "$LOG_FILE"

    if (( EUID == 0 )) && [ -n "$YEAR" ]; then
        fix_owner "$LOG_DIR/$YEAR" "$LOG_DIR/$YEAR/$MONTH_DIR" "$LOG_FILE"
    fi
}

# Run function after every command
[[ $PROMPT_COMMAND == *log_command* ]] ||  # add once; avoid export
    PROMPT_COMMAND="log_command${PROMPT_COMMAND:+; $PROMPT_COMMAND}"