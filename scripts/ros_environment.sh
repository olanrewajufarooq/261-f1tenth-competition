#!/usr/bin/env bash
# Shared by the setup, launch, stop, and smoke-test entry points.

source_environment() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

require_lyrical_shell() {
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != lyrical ]]; then
    echo "ROS_DISTRO=$ROS_DISTRO conflicts with Lyrical. Open a fresh terminal without sourcing another ROS distribution." >&2
    return 1
  fi
  local prefixes prefix
  local -a prefix_entries
  for prefixes in "${AMENT_PREFIX_PATH:-}" "${CMAKE_PREFIX_PATH:-}" "${COLCON_PREFIX_PATH:-}"; do
    IFS=: read -r -a prefix_entries <<< "$prefixes"
    for prefix in "${prefix_entries[@]}"; do
      if [[ "$prefix" == /opt/ros/* && "$prefix" != /opt/ros/lyrical && "$prefix" != /opt/ros/lyrical/* ]]; then
        echo "Conflicting ROS prefix: $prefix. Open a fresh terminal without sourcing another ROS distribution." >&2
        return 1
      fi
    done
  done
}

source_ros_lyrical() {
  require_lyrical_shell || return 1
  if [[ ! -f /opt/ros/lyrical/setup.bash ]]; then
    echo "ROS 2 Lyrical was not found at /opt/ros/lyrical/setup.bash. Follow docs/INSTALL.md." >&2
    return 1
  fi
  source_environment /opt/ros/lyrical/setup.bash
}

workspace_platform() {
  /usr/bin/python3 -c 'import sys; print(f"lyrical:{sys.version_info.major}.{sys.version_info.minor}")'
}

check_workspace_platform() {
  local workspace="$1" mode="$2" expected actual system_python venv_python created_python activated_prefix expected_prefix
  expected="$(workspace_platform)"
  if [[ -f "$workspace/.platform" ]]; then
    actual="$(cat "$workspace/.platform")"
    if [[ "$actual" != "$expected" ]]; then
      echo "Incompatible workspace platform: $actual (expected $expected). Move .venv, build, install, log, and .platform out of starter_ws, then rerun setup_workspace.sh." >&2
      return 1
    fi
  elif [[ -d "$workspace/build" || -d "$workspace/install" || -d "$workspace/log" ]]; then
    echo "Unmarked workspace outputs found. Move .venv, build, install, and log out of starter_ws, then rerun setup_workspace.sh." >&2
    return 1
  elif [[ "$mode" != setup ]]; then
    echo "Workspace has not been set up for Lyrical. Run ./scripts/setup_workspace.sh first." >&2
    return 1
  fi
  if [[ -d "$workspace/.venv" ]]; then
    if [[ ! -x "$workspace/.venv/bin/python" ]]; then
      echo "Incomplete virtual environment. Move starter_ws/.venv aside and rerun setup_workspace.sh." >&2
      return 1
    fi
    system_python="$(/usr/bin/python3 -c 'import sys; print(sys.version_info[:2])')"
    venv_python="$("$workspace/.venv/bin/python" -c 'import sys; print(sys.version_info[:2])')" || return 1
    if [[ "$system_python" != "$venv_python" ]]; then
      echo "Incompatible virtual-environment Python. Move starter_ws/.venv aside and rerun setup_workspace.sh with the system Python." >&2
      return 1
    fi
    created_python="$(sed -n 's/^version = \([0-9]*\.[0-9]*\).*/\1/p' "$workspace/.venv/pyvenv.cfg")"
    if [[ "lyrical:$created_python" != "$expected" ]]; then
      echo "Virtual environment was created for Python $created_python, not ${expected#lyrical:}. Move starter_ws/.venv aside and rerun setup_workspace.sh." >&2
      return 1
    fi
    expected_prefix="$(realpath "$workspace/.venv")"
    activated_prefix="$(
      source_environment "$workspace/.venv/bin/activate"
      python -c 'import os, sys; print(os.path.realpath(sys.prefix))'
    )" || {
      echo "Relocated or unusable virtual environment. Move starter_ws/.venv aside and rerun setup_workspace.sh." >&2
      return 1
    }
    if [[ "$activated_prefix" != "$expected_prefix" ]]; then
      echo "Relocated virtual environment activates $activated_prefix instead of $expected_prefix. Move starter_ws/.venv aside and rerun setup_workspace.sh." >&2
      return 1
    fi
    if ! grep -Eq '^include-system-site-packages = true$' "$workspace/.venv/pyvenv.cfg"; then
      echo "Virtual environment cannot access system ROS packages. Move starter_ws/.venv aside and rerun setup_workspace.sh." >&2
      return 1
    fi
  fi
  if [[ "$mode" != setup && ( ! -f "$workspace/install/setup.bash" || ! -f "$workspace/.venv/bin/activate" ) ]]; then
    echo "Workspace setup is incomplete. Run ./scripts/setup_workspace.sh first." >&2
    return 1
  fi
}
