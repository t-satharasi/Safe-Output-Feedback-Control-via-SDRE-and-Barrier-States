#!/bin/bash

export PX4_HOME_LAT=29.6282703
export PX4_HOME_LON=-82.3606036
export PX4_HOME_ALT=30.861857569478296
export HEADLESS=1

PX4_DIR=~/PX4-Autopilot
READY_STRING="Ready for takeoff!"

launch_px4() {
    local instance=$1
    local autostart=$2
    local model=$3
    local pose=$4
    local logfile="/tmp/px4_instance_${instance}.log"

    echo "[*] Launching PX4 instance $instance..."

    if [ -n "$pose" ]; then
        PX4_HOME_LAT=$PX4_HOME_LAT \
        PX4_HOME_LON=$PX4_HOME_LON \
        PX4_HOME_ALT=$PX4_HOME_ALT \
        PX4_SYS_AUTOSTART=$autostart \
        PX4_GZ_MODEL_POSE="$pose" \
        PX4_SIM_MODEL=$model \
        $PX4_DIR/build/px4_sitl_default/bin/px4 -i $instance > "$logfile" 2>&1 &
    else
        PX4_HOME_LAT=$PX4_HOME_LAT \
        PX4_HOME_LON=$PX4_HOME_LON \
        PX4_HOME_ALT=$PX4_HOME_ALT \
        PX4_SYS_AUTOSTART=$autostart \
        PX4_SIM_MODEL=$model \
        $PX4_DIR/build/px4_sitl_default/bin/px4 -i $instance > "$logfile" 2>&1 &
    fi

    echo "PID: $!" # return PID
}

wait_for_ready() {
    local instance=$1
    local logfile="/tmp/px4_instance_${instance}.log"
    local timeout=120  # seconds
    local elapsed=0

    echo "[*] Waiting for instance $instance to be ready..."

    while [ $elapsed -lt $timeout ]; do
        if grep -q "$READY_STRING" "$logfile" 2>/dev/null; then
            echo "[+] Instance $instance is ready!"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "[!] Timeout waiting for instance $instance. Check $logfile"
    return 1
}

# --- Launch sequence ---

cd $PX4_DIR

# Instance 1 - loads Gazebo world
launch_px4 1 4002 gz_x500_depth ""
wait_for_ready 1 || exit 1

# Instance 2
launch_px4 2 4001 gz_x500_depth "0,4"
wait_for_ready 2 || exit 1

# Instance 3
launch_px4 3 4001 gz_x500_depth "0,8"
wait_for_ready 3 || exit 1

echo "[+] All instances ready!"

# Keep script alive so background processes don't get killed
# (remove this if you want the script to exit and leave them running)
wait
