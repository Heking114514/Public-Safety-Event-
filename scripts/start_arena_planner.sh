#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_DIR}"
BUILD_LOCK_HELPER="${SCRIPT_DIR}/build_lock.sh"
PROCESS_HELPER="${SCRIPT_DIR}/navigation_processes.sh"
DEFAULT_MAP="${WORKSPACE_DIR}/src/arena_path_planner/config/arena_map.yaml"
MAP_FILE="${ARENA_MAP:-${DEFAULT_MAP}}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"
PGREP_COMMAND="${ARENA_PGREP_COMMAND:-pgrep}"
ROS2_COMMAND="${ARENA_ROS2_COMMAND:-ros2}"

log() {
  printf '[arena-planner] %s\n' "$*"
}

fail() {
  printf '[arena-planner] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '用法: %s [--map /绝对或相对路径/map.yaml]\n' "${BASH_SOURCE[0]}"
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      MAP_FILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done
if [[ "${MAP_FILE}" != /* ]]; then
  MAP_FILE="${WORKSPACE_DIR}/${MAP_FILE}"
fi
[[ -f "${MAP_FILE}" ]] || { printf '地图文件不存在: %s\n' "${MAP_FILE}" >&2; exit 1; }
[[ -f "${BUILD_LOCK_HELPER}" ]] || fail "构建锁脚本不存在: ${BUILD_LOCK_HELPER}"
[[ -f "${PROCESS_HELPER}" ]] || fail "进程管理脚本不存在: ${PROCESS_HELPER}"
export ARENA_MAP="${MAP_FILE}"
log "Map: ${ARENA_MAP}"

# ROS 2 generated setup files read optional variables before defining them,
# which is incompatible with nounset on some Humble installations.
set +u
source "${ROS_SETUP}"
set -u
# shellcheck disable=SC1090
source "${BUILD_LOCK_HELPER}"
# shellcheck disable=SC1090
source "${PROCESS_HELPER}"

log "Waiting for exclusive workspace build access"
acquire_workspace_build_lock || fail "无法获取工作空间构建锁"
(cd "${WORKSPACE_DIR}" && colcon build --symlink-install --packages-select arena_path_planner)
release_workspace_build_lock
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

BACKEND_PID=""
# A planner process owns its map at startup. Wait for every old launch/backend
# process to exit before publishing the new service with this UI's map.
stop_arena_planner_processes || fail "旧规划后端无法停止"
"${ROS2_COMMAND}" launch arena_path_planner arena_path_planner.launch.py config:="${MAP_FILE}" &
BACKEND_PID=$!
cleanup() {
  if [[ -n "${BACKEND_PID}" ]]; then
    kill "${BACKEND_PID}" 2>/dev/null || true
    wait "${BACKEND_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

for _ in {1..50}; do
  if "${ROS2_COMMAND}" service type /arena_path_planner/plan >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

python3 "${SCRIPT_DIR}/arena_route_frontend.py"
