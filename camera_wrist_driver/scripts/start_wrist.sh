#!/usr/bin/env bash
# start_wrist.sh - systemd wrapper: load ROS environment then exec the wrist camera node.
# Usage: start_wrist.sh <left|right>
#
# Reads the camera serial from /etc/realsense-wrist/wrist_<side>.env
# (WRIST_SERIAL=...), sources the required ROS environments in the
# documented order, then execs the node so systemd tracks the real
# process (SIGTERM propagation).
set -eo pipefail

SIDE="${1:-}"
if [[ "$SIDE" != "left" && "$SIDE" != "right" ]]; then
    echo "ERROR: usage: $0 <left|right>" >&2
    exit 2
fi

ENV_FILE="/etc/realsense-wrist/wrist_${SIDE}.env"
if [[ ! -f "$ENV_FILE" ]]; then
    echo "ERROR: environment file not found: $ENV_FILE (run install.sh first)" >&2
    exit 2
fi
# shellcheck disable=SC1090
. "$ENV_FILE"

if [[ -z "${WRIST_SERIAL:-}" ]]; then
    echo "ERROR: WRIST_SERIAL is empty in $ENV_FILE" >&2
    exit 2
fi

# Load ROS environment (order matters, per the top-level README).
# ROS setup scripts reference unset variables (e.g. AMENT_TRACE_SETUP_FILES),
# so nounset (-u) must stay off while sourcing them.
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/humanoid/install/setup.bash ]]; then
    source /opt/humanoid/install/setup.bash
fi
if [[ -f "${HOME}/xos/setup.bash" ]]; then
    source "${HOME}/xos/setup.bash"
fi

# Run from the source tree directly so no colcon build is required:
# scripts/.. is this project (camera_wrist_driver), python/ holds the package.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${PROJECT_DIR}/python:${PYTHONPATH:-}"

# Wait for the robot network before starting the node (same pattern as the
# orbbec camera services). With ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, FastDDS
# enumerates the interfaces once at startup; if the 41-segment NIC is not up
# yet the UDP transport is disabled for good ("All whitelist interfaces were
# filtered out") and the topics stay invisible to the whole robot - FastDDS
# does not retry. systemd After=network.target does NOT mean the NIC is up.
for _ in $(seq 1 50); do
    if ping -c 1 -W 1 192.168.41.1 >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

echo "[$(date '+%F %T')] Starting wrist camera (${SIDE}) SN=${WRIST_SERIAL} repo=${PROJECT_DIR}"

# WRIST_SERIAL/WRIST_SIDE/WRIST_ROTATE/WRIST_WIDTH/WRIST_HEIGHT/WRIST_FPS/
# WRIST_COMPRESSED/WRIST_JPEG_QUALITY/WRIST_DEPTH_MAX are read by the node
# as parameter fallbacks.
export WRIST_SERIAL
export WRIST_SIDE="${SIDE}"
export WRIST_ROTATE="${WRIST_ROTATE:-0}"
export WRIST_WIDTH="${WRIST_WIDTH:-640}"
export WRIST_HEIGHT="${WRIST_HEIGHT:-480}"
export WRIST_FPS="${WRIST_FPS:-15}"
export WRIST_COMPRESSED="${WRIST_COMPRESSED:-1}"
export WRIST_JPEG_QUALITY="${WRIST_JPEG_QUALITY:-80}"
export WRIST_DEPTH_MAX="${WRIST_DEPTH_MAX:-10.0}"

exec python3 -m camera_wrist_driver.camera_wrist_driver_node
