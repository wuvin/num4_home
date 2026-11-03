#!/bin/bash

# Shell script to execute inside container and start ROS 2 bag recording
# of predetermined topics. See code for list of topics.
#
# Usage:
#   record_num4_bag.sh          record bag
#   record_num4_bag.sh preset   use topics denoted by preset tag
#
# Also supports flags or options for `ros2 bag record`.
#
# Do not forget to `chmod +x record_num4_bag.sh`.
#
# Contact:      wu.kevi@northeastern.edu
# Last Updated: November 02, 2026

# Configure default values
STORAGE="mcap"
BAG_LABEL="dataset"
FLAG_OVERRIDE=""
TOPIC_DISCOVERY_TIME=0

# Configure topics
get_topics() {
    case "$1" in
        color)
            echo /camera/camera/color/camera_info
            echo /camera/camera/color/image_raw/compressed
            ;;
        depth)
            echo /camera/camera/aligned_depth_to_color/camera_info
            echo /camera/camera/aligned_depth_to_color/image_raw/compressedDepth
            ;;
        extrinsics)
            echo /camera/camera/extrinsics/depth_to_accel
            echo /camera/camera/extrinsics/depth_to_color
            echo /camera/camera/extrinsics/depth_to_gyro
            ;;
        lidar)
            echo /cloud_registered
            echo /livox/lidar
            ;;
        slam)
            echo /aft_mapped_to_init
            ;;
        imufilt)
            echo /madgwick/d455/imu
            echo /madgwick/mid360/imu
            echo /mavros/imu/data      # post-AHRS or state estimator
            ;;
        imuraw)
            echo /camera/camera/accel/sample
            echo /camera/camera/gyro/sample
            echo /camera/camera/imu
            echo /livox/imu
            echo /mavros/imu/data_raw  # raw sensor meas., pre-AHRS
            echo /mavros/imu/mag       # raw magnetometer (compass)
            ;;
        imu)
            get_topics imufilt
            get_topics imuraw
            ;;
        rc)
            echo /mavros/rc/in        # incoming raw PWM read by FCU
            echo /mavros/rc/out       # PWM from FCU to servo motor
            echo /mavros/rc/override  # injected RC inputs or PWM values
            ;;
        mp)
            echo /m4/drive_cmd
            echo /m4/joint_states
            echo /m4/wheel_states
            ;;
        camera)
            get_topics color
            get_topics depth
            ;;
        realsense)
            get_topics camera
            get_topics extrinsics
            echo /madgwick/d455/imu
            echo /camera/camera/accel/sample
            echo /camera/camera/gyro/sample
            echo /camera/camera/imu
            ;;
        livox)
            get_topics lidar
            echo /madgwick/mid360/imu
            echo /livox/imu
            ;;
        tf)
            echo /tf
            echo /tf_static
            ;;
        mavros)
            echo /mavros/battery
            echo /mavros/local_position/odom
            echo /mavros/local_position/pose
            echo /mavros/local_position/velocity_body
            echo /mavros/setpoint_accel/accel
            echo /mavros/setpoint_attitude/cmd_vel
            echo /mavros/setpoint_position/global
            echo /mavros/setpoint_position/global_to_local
            echo /mavros/setpoint_position/local
            echo /mavros/setpoint_raw/global
            echo /mavros/setpoint_trajectory/desired
            echo /mavros/setpoint_velocity/cmd_vel
            ;;
        other)
            echo /mavros/vision_pose/pose
            echo /path
            ;;
        default)
            get_topics realsense
            get_topics livox
            get_topics imu
            get_topics mp
            get_topics tf
            get_topics mavros
            get_topics other
            ;;
        *)
            echo "Unknown input: $1." >&2;
            return 1
            ;;
    esac
}

# Parse flags
you_missed_a_spot() {
    [[ $# -ge 2 && "$2" != -* ]] || {
        echo "Missing value for $1" >&2
        exit 1
    }
}

POSITIONAL_ARGS=()
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --label)
            you_missed_a_spot "$@"
            BAG_LABEL="$2"
            shift 2
            ;;
        --wait)
            you_missed_a_spot "$@"
            TOPIC_DISCOVERY_TIME="$2"
            shift 2
            ;;
        -s)
            you_missed_a_spot "$@"
            STORAGE="$2"
            shift 2
            ;;
        -o)
            you_missed_a_spot "$@"
            FLAG_OVERRIDE="$2"
            shift 2
            ;;
        [!-]*)
            POSITIONAL_ARGS+=("$1")
            shift 1
            ;;
        -*) 
            # other flags for `ros2 bag record`
            EXTRA_ARGS+=("$1")
            if [[ -n "$2" && "$2" != -* ]]; then
                EXTRA_ARGS+=("$2")
                shift 2
            else
                shift 1
            fi
            ;;
        *)
            # empty
            shift 1
            ;;
    esac
done

# Get date value
YEAR=$(date +%Y)
MONTH_NUM=$(date +%m)
MONTH_STR="${MONTH_NUM} - $(date +%B)"
DAY=$(date +%d)
DATE_TAG=$(date +%Y%m%d)

# Set default output path
DEFAULT_DIR="/home/num4/data/bag/${YEAR}/${MONTH_STR}/${DAY}"
OUTPUT_PATH="${DEFAULT_DIR}/run${DATE_TAG}_${BAG_LABEL}"
OUTPUT_PATH="${FLAG_OVERRIDE:-$OUTPUT_PATH}"

# Get unique list of target topics
RAW_TOPICS=()

if [[ ${#POSITIONAL_ARGS[@]} -eq 0 ]]; then
    POSITIONAL_ARGS=("default")
fi

for preset in "${POSITIONAL_ARGS[@]}"; do
    # Validate preset args before collecting
    get_topics "$preset" >/dev/null || exit 1

    # Collect topics
    while IFS= read -r topic; do
        [[ -n "$topic" ]] && RAW_TOPICS+=("$topic")
    done < <(get_topics "$preset")
done

readarray -t TARGET_TOPICS < <(
    printf '%s\n' "${RAW_TOPICS[@]}" | awk '!seen[$0]++'
)

# Display target topics
echo -e "\nTarget Topics:"
for topic in "${TARGET_TOPICS[@]}"; do
    echo "  - $topic"
done
echo -e "\n"

# Set output path
read -p "Enter output path (${OUTPUT_PATH}):  " USERIN
USERIN="${USERIN/#\~/$HOME}"
OUTPUT_PATH="${USERIN:-$OUTPUT_PATH}"

# Check if already exists
looks_like_bag() {  # $1=outdir
    [[ -f "$1/metadata.yaml" ]] \
        || compgen -G "$1/*.mcap" >/dev/null \
        || compgen -G "$1/*.db3" >/dev/null
}

if [[ -e "${OUTPUT_PATH}" ]]; then
    echo -e "\nOutput path already exists."
    while true; do
        read -p " Overwrite? (y/n): " YN
        case $YN in
            [Yy]* )
                if looks_like_bag "${OUTPUT_PATH}"; then
                    rm -rf -- "${OUTPUT_PATH}"
                    echo "Removed existing bag directory."
                    break
                else
                    echo "Existing directory does not match bag format."
                    exit 1
                fi
                ;;
            [Nn]* )
                exit 0
                ;;
            *)
                echo "Unknown input: $YN."
                continue
                ;;
        esac
    done
fi

# Check for missing topics
ACTIVE_TOPICS=$(ros2 topic list 2>/dev/null)
MISSING_TOPICS=()

for topic in "${TARGET_TOPICS[@]}"; do
    if ! echo "$ACTIVE_TOPICS" | grep -qx "$topic"; then
        MISSING_TOPICS+=("$topic")
    fi
done

if [[ ${#MISSING_TOPICS[@]} -gt 0 ]]; then  # try again
    sleep ${TOPIC_DISCOVERY_TIME}
    ACTIVE_TOPICS=$(ros2 topic list 2>/dev/null)
    MISSING_TOPICS=()
    for topic in "${TARGET_TOPICS[@]}"; do
        if ! echo "$ACTIVE_TOPICS" | grep -qx "$topic"; then
            MISSING_TOPICS+=("$topic")
        fi
    done
fi

if [[ ${#MISSING_TOPICS[@]} -gt 0 ]]; then
    echo -e "\nFailed to find all target topics.\n\nMissing Topics:"
    for topic in "${MISSING_TOPICS[@]}"; do
        echo "  - $topic"
    done

    while true; do
        read -p "Proceed? (y/n):  " YN
        case $YN in
            [Yy]* )
                break
                ;;
            [Nn]* )
                exit 0
                ;;
            * )
                echo "Unknown input: $YN."
                continue
                ;;
        esac
    done
else
    echo "Found all target topics."
fi

# Execute
ros2 bag record -o "${OUTPUT_PATH}" -s "${STORAGE}" "${EXTRA_ARGS[@]}" \
    "${TARGET_TOPICS[@]}"