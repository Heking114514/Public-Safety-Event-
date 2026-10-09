#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${LOOPBACK_ROS_SETUP:-${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}}"
ROS2_COMMAND="${LOOPBACK_ROS2_COMMAND:-ros2}"
PGREP_COMMAND="${LOOPBACK_PGREP_COMMAND:-pgrep}"
GUI_PORT="${LOOPBACK_GUI_PORT:-8765}"
GUI_ENABLED=true

log() {
  printf '[loopback-sim] %s\n' "$*"
}

fail() {
  printf '[loopback-sim] ERROR: %s\n' "$*" >&2
  exit 1
}

[[ -r "${ROS_SETUP}" ]] || fail "ROS 环境不存在: ${ROS_SETUP}"
[[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]] ||
  fail "工作空间尚未构建: ${WORKSPACE_ROOT}/install/setup.bash"

# ROS setup files use optional variables without `${VAR:-...}`. Temporarily
# relax nounset while sourcing them, then restore the script's strict mode.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
# shellcheck disable=SC1090
source "${SCRIPT_DIR}/navigation_processes.sh"

launch_args=("$@")
has_gui_port=false
for argument in "${launch_args[@]}"; do
  if [[ "${argument}" == gui_port:=* ]]; then
    GUI_PORT="${argument#gui_port:=}"
    has_gui_port=true
  elif [[ "${argument}" == gui:=false ]]; then
    GUI_ENABLED=false
  fi
done

stop_loopback_processes

# A stale GUI can survive after its ROS node has exited. Recheck the requested
# port and only terminate a process that is clearly this workspace's loopback
# GUI; unrelated services are left untouched and reported to the operator.
if [[ "${GUI_ENABLED}" == true ]] && command -v ss >/dev/null 2>&1; then
  mapfile -t port_pids < <(
    ss -ltnpH 2>/dev/null |
      awk -v port=":${GUI_PORT}" '$4 ~ port"$" {while (match($0, /pid=[0-9]+/)) {print substr($0, RSTART + 4, RLENGTH - 4); $0 = substr($0, RSTART + RLENGTH)}}' |
      sort -u
  )
  for pid in "${port_pids[@]}"; do
    [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/cmdline" ]] || continue
    cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
    if [[ "${cmdline}" == *"${WORKSPACE_ROOT}/install/vehicle_loopback_sim/"*vehicle_loopback_gui_node* ]]; then
      log "Stopping stale loopback GUI on port ${GUI_PORT} (pid=${pid})"
      kill -TERM "${pid}" 2>/dev/null || true
    else
      fail "GUI 端口 ${GUI_PORT} 被其他进程占用: pid=${pid}"
    fi
  done
fi

if [[ "${has_gui_port}" == false ]]; then
  launch_args+=("gui_port:=${GUI_PORT}")
fi

log "Starting planner/control loopback (GUI port ${GUI_PORT})"
exec "${ROS2_COMMAND}" launch vehicle_loopback_sim planner_control_loopback.launch.py "${launch_args[@]}"
