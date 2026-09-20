#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_DIR}"
BUILD_LOCK_HELPER="${SCRIPT_DIR}/build_lock.sh"
PROCESS_HELPER="${SCRIPT_DIR}/navigation_processes.sh"
CONFIG_HELPER="${SCRIPT_DIR}/navigation_config.py"
CONFIG_SHELL_HELPER="${SCRIPT_DIR}/navigation_config.sh"
STARTUP_CONFIG="${NAVIGATION_STARTUP_CONFIG:-${WORKSPACE_ROOT}/config/navigation_startup.yaml}"
RECORDING_CONFIG="${NAVIGATION_RECORDING_CONFIG:-${WORKSPACE_ROOT}/config/navigation_recording.yaml}"
DEFAULT_MAP="${WORKSPACE_DIR}/src/arena_path_planner/config/arena_map.yaml"
MAP_FILE="${ARENA_MAP:-${DEFAULT_MAP}}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"
PGREP_COMMAND="${ARENA_PGREP_COMMAND:-pgrep}"
ROS2_COMMAND="${ARENA_ROS2_COMMAND:-ros2}"
ODOM_TOPIC="/odometry/local_map"
PLANNER_SERVICE="/arena_path_planner/plan"
PLANNER_STARTUP_TIMEOUT_SEC="${ARENA_PLANNER_STARTUP_TIMEOUT_SEC:-30}"
PLANNER_SERVICE_CLEAR_TIMEOUT_SEC="${ARENA_PLANNER_SERVICE_CLEAR_TIMEOUT_SEC:-10}"

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
[[ -f "${CONFIG_SHELL_HELPER}" ]] || fail "导航配置脚本不存在: ${CONFIG_SHELL_HELPER}"
# shellcheck disable=SC1090
source "${CONFIG_SHELL_HELPER}"
load_navigation_config
export ARENA_MAP="${MAP_FILE}"
export NAVIGATION_ODOM_TOPIC="${ODOM_TOPIC}"
log "Map: ${ARENA_MAP}"
log "Odometry topic: ${NAVIGATION_ODOM_TOPIC}"

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

[[ "${PLANNER_STARTUP_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] ||
  fail "ARENA_PLANNER_STARTUP_TIMEOUT_SEC 必须是正整数"
[[ "${PLANNER_SERVICE_CLEAR_TIMEOUT_SEC}" =~ ^[1-9][0-9]*$ ]] ||
  fail "ARENA_PLANNER_SERVICE_CLEAR_TIMEOUT_SEC 必须是正整数"

wait_for_planner_service_clear() {
  local deadline=$((SECONDS + PLANNER_SERVICE_CLEAR_TIMEOUT_SEC))

  while ((SECONDS < deadline)); do
    if ! "${ROS2_COMMAND}" service type "${PLANNER_SERVICE}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

BACKEND_PID=""
# A planner process owns its map at startup. Clear both old processes and the
# old graph entry before publishing the new service with this UI's map.
stop_arena_planner_processes || fail "旧规划后端无法停止"
"${ROS2_COMMAND}" daemon stop >/dev/null 2>&1 || true
wait_for_planner_service_clear ||
  fail "旧规划后端的 service ${PLANNER_SERVICE} 在 ${PLANNER_SERVICE_CLEAR_TIMEOUT_SEC}s 内未消失"

"${ROS2_COMMAND}" launch arena_path_planner arena_path_planner.launch.py config:="${MAP_FILE}" &
BACKEND_PID=$!
BACKEND_START_TIME="$(process_start_time "${BACKEND_PID}" 2>/dev/null || true)"
BACKEND_NODE_PID=""
BACKEND_NODE_START_TIME=""
cleanup() {
  local exit_code=$?
  local current_start=""

  trap - EXIT INT TERM
  if [[ -n "${BACKEND_PID}" && -n "${BACKEND_START_TIME}" ]]; then
    current_start="$(process_start_time "${BACKEND_PID}" 2>/dev/null || true)"
    if [[ "${current_start}" == "${BACKEND_START_TIME}" ]] &&
      process_is_running "${BACKEND_PID}"; then
      kill -TERM "${BACKEND_PID}" 2>/dev/null || true
    fi
  fi
  if [[ -n "${BACKEND_NODE_PID}" && -n "${BACKEND_NODE_START_TIME}" ]]; then
    current_start="$(process_start_time "${BACKEND_NODE_PID}" 2>/dev/null || true)"
    if [[ "${current_start}" == "${BACKEND_NODE_START_TIME}" ]] &&
      process_is_running "${BACKEND_NODE_PID}"; then
      kill -TERM "${BACKEND_NODE_PID}" 2>/dev/null || true
    fi
  fi
  # ros2 launch can lose its child when the parent shell exits. Re-discover
  # and stop the actual planner node as a final cleanup step.
  stop_arena_planner_processes || log "Some planner processes remained during cleanup"
  if [[ -n "${BACKEND_PID}" ]]; then
    wait "${BACKEND_PID}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

log "Waiting up to ${PLANNER_STARTUP_TIMEOUT_SEC}s for the new planner backend"
PLANNER_READY=0
STARTUP_DEADLINE=$((SECONDS + PLANNER_STARTUP_TIMEOUT_SEC))
while ((SECONDS < STARTUP_DEADLINE)); do
  if ! process_is_running "${BACKEND_PID}"; then
    BACKEND_EXIT_STATUS=0
    if wait "${BACKEND_PID}"; then
      BACKEND_EXIT_STATUS=0
    else
      BACKEND_EXIT_STATUS=$?
    fi
    fail "规划后端 launch 进程提前退出，状态码 ${BACKEND_EXIT_STATUS}"
  fi

  PLANNER_NODE_PIDS=()
  mapfile -t PLANNER_NODE_PIDS < <(find_arena_planner_node_processes)
  if ((${#PLANNER_NODE_PIDS[@]} > 0)); then
    BACKEND_NODE_PID="${PLANNER_NODE_PIDS[0]}"
    BACKEND_NODE_START_TIME="$(process_start_time "${BACKEND_NODE_PID}" 2>/dev/null || true)"
    CURRENT_NODE_START_TIME="$(process_start_time "${BACKEND_NODE_PID}" 2>/dev/null || true)"
    if [[ -n "${BACKEND_NODE_START_TIME}" ]] &&
      process_is_running "${BACKEND_NODE_PID}" &&
      [[ "${CURRENT_NODE_START_TIME}" == "${BACKEND_NODE_START_TIME}" ]] &&
      "${ROS2_COMMAND}" service type "${PLANNER_SERVICE}" >/dev/null 2>&1; then
      PLANNER_READY=1
      break
    fi
  fi
  sleep 0.1
done

if ((PLANNER_READY == 0)); then
  fail "规划后端在 ${PLANNER_STARTUP_TIMEOUT_SEC}s 内未出现真实节点和 service ${PLANNER_SERVICE}"
fi

log "Planner backend ready: node PID ${BACKEND_NODE_PID}, service ${PLANNER_SERVICE}"
python3 "${SCRIPT_DIR}/arena_route_frontend.py"
