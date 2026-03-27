#!/bin/bash
# Launch the Dyno Qt GUI (and bridge_ros2 subprocess).
# Run from the repo root: bash src/interface_bridges/ros2/run_gui.sh [gui args...]
#
# Why this wrapper exists:
#   bridge_ros2 must run as root (raw EtherCAT socket).  Fast-DDS shared memory
#   segments are not accessible across root/user boundaries, causing silent
#   message drops.  Forcing UDP transport on both sides fixes this.
#   PYTHONPATH must also be forwarded so rclpy is visible under sudo.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PROFILES="$SCRIPT_DIR/fastdds_no_shm.xml"

source /opt/ros/humble/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILES"

exec sudo \
    PYTHONPATH="$PYTHONPATH" \
    FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILES" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    python3 "$SCRIPT_DIR/dyno_gui.py" "$@"
