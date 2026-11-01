#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WS="$REPO_ROOT/starter_ws"

# shellcheck source=ros_environment.sh
source "$SCRIPT_DIR/ros_environment.sh"
source_ros_lyrical
check_workspace_platform "$WS" launch
source_environment "$WS/.venv/bin/activate"
source_environment "$WS/install/setup.bash"

exec ros2 launch competition_bringup competition_sim_launch.py
