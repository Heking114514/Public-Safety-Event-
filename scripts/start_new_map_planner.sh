#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_LOCK_HELPER="${SCRIPT_DIR}/build_lock.sh"
DEFAULT_MAP="${WORKSPACE_ROOT}/src/new_map_planner/config/arena.yaml"
MAP_FILE="${NEW_MAP_PLANNER_MAP:-${DEFAULT_MAP}}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"

log() {
  printf '[new-map-planner] %s\n' "$*"
}

fail() {
  printf '[new-map-planner] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '用法: %s [--map MAP.yaml] [--no-ui]\n' "${BASH_SOURCE[0]}"
}

START_UI=true
while (($# > 0)); do
  case "$1" in
    --map)
      (($# >= 2)) || fail "--map 需要一个 YAML 路径"
      MAP_FILE="$2"
      shift 2
      ;;
    --no-ui)
      START_UI=false
      shift
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
  MAP_FILE="${WORKSPACE_ROOT}/${MAP_FILE}"
fi
[[ -f "${MAP_FILE}" ]] || fail "地图文件不存在: ${MAP_FILE}"
[[ -r "${ROS_SETUP}" ]] || fail "ROS 环境不存在: ${ROS_SETUP}"
[[ -f "${BUILD_LOCK_HELPER}" ]] || fail "构建锁脚本不存在: ${BUILD_LOCK_HELPER}"

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
# shellcheck disable=SC1090
source "${BUILD_LOCK_HELPER}"

log "等待工作空间构建锁"
acquire_workspace_build_lock || fail "无法获取工作空间构建锁"
(
  cd "${WORKSPACE_ROOT}"
  colcon build --symlink-install --packages-select \
    vehicle_config mission_control_interfaces visual_navigation new_map_planner \
    navigation_mission_manager
)
release_workspace_build_lock

set +u
# shellcheck disable=SC1090
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

if ros2 service type /new_map_planner/plan >/dev/null 2>&1; then
  fail "已有 /new_map_planner/plan 服务；请先关闭旧的新规划器"
fi
if ros2 service type /arena_path_planner/plan >/dev/null 2>&1; then
  fail "检测到旧 /arena_path_planner/plan 服务；请先关闭旧规划器"
fi
if ros2 service type /navigation_mission/control >/dev/null 2>&1; then
  fail "已有 /navigation_mission/control 服务；请先关闭旧任务节点"
fi

log "地图: ${MAP_FILE}"
log "模式: 实车自主任务（不启动仿真）"
export PYTHONDONTWRITEBYTECODE=1
exec ros2 launch navigation_mission_manager mission_navigation.launch.py \
  map_file:="${MAP_FILE}" start_ui:="${START_UI}"
