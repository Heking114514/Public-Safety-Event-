#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_LOCK_HELPER="${SCRIPT_DIR}/build_lock.sh"
BUILD_MANIFEST_HELPER="${SCRIPT_DIR}/navigation_run_manifest.py"
BUILD_MANIFEST="${NEW_MAP_PLANNER_BUILD_MANIFEST:-${WORKSPACE_ROOT}/build/new_map_planner_runtime_manifest.json}"
DEFAULT_MAP="${WORKSPACE_ROOT}/src/new_map_planner/config/arena.yaml"
MAP_FILE="${NEW_MAP_PLANNER_MAP:-${DEFAULT_MAP}}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"
COLCON_COMMAND="${NEW_MAP_PLANNER_COLCON_COMMAND:-colcon}"
ROS2_COMMAND="${NEW_MAP_PLANNER_ROS2_COMMAND:-ros2}"
STARTUP_CONFIG="${WORKSPACE_ROOT}/config/navigation_startup.yaml"
RECORDING_CONFIG="${WORKSPACE_ROOT}/config/navigation_recording.yaml"
CONFIG_HELPER="${SCRIPT_DIR}/navigation_config.py"

log() {
  printf '[new-map-planner] %s\n' "$*"
}

cleanup_previous_planner() {
  local patterns=(
    "new_map_planner_node"
    "new_map_planner_ui"
    "mission_manager_node"
  )
  local pattern pid
  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [[ -n "${pid}" ]] || continue
      if [[ "${pid}" == "$$" || "${pid}" == "${PPID}" ]]; then
        continue
      fi
      log "关闭旧进程: ${pattern} (pid=${pid})"
      kill "${pid}" 2>/dev/null || true
    done < <(pgrep -f "${WORKSPACE_ROOT}/install/.*/${pattern}" || true)
  done
  sleep 0.3
}

fail() {
  printf '[new-map-planner] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '用法: %s [--map MAP.yaml] [--no-ui] [--no-build]\n' "${BASH_SOURCE[0]}"
}

START_UI=true
BUILD_IF_NEEDED=true
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
    --no-build)
      BUILD_IF_NEEDED=false
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
[[ -x "${BUILD_MANIFEST_HELPER}" ]] || fail "构建清单工具不存在: ${BUILD_MANIFEST_HELPER}"
[[ -f "${STARTUP_CONFIG}" ]] || fail "启动配置不存在: ${STARTUP_CONFIG}"
[[ -f "${RECORDING_CONFIG}" ]] || fail "录包配置不存在: ${RECORDING_CONFIG}"

ROS_DOMAIN_ID_CONFIG="$(${CONFIG_HELPER} --startup "${STARTUP_CONFIG}" \
  --recording "${RECORDING_CONFIG}" | awk -F '\t' '$1 == "ros_domain_id" {print $2}')" ||
  fail "无法读取 ROS domain 配置"
[[ "${ROS_DOMAIN_ID_CONFIG}" =~ ^[0-9]+$ ]] || fail "ROS domain 配置无效"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CONFIG}"
log "ROS domain: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
# shellcheck disable=SC1090
source "${BUILD_LOCK_HELPER}"

log "等待工作空间构建锁"
acquire_workspace_build_lock || fail "无法获取工作空间构建锁"

PLANNER_ARTIFACTS=(
  "planner_node=${WORKSPACE_ROOT}/install/new_map_planner/lib/new_map_planner/new_map_planner_node"
  "planner_ui=${WORKSPACE_ROOT}/install/new_map_planner/lib/new_map_planner/new_map_planner_ui"
  "mission_manager=${WORKSPACE_ROOT}/install/navigation_mission_manager/lib/navigation_mission_manager/mission_manager_node"
  "final_navigation=${WORKSPACE_ROOT}/install/final_navigation/lib/final_navigation/final_navigation"
  "mission_typesupport=${WORKSPACE_ROOT}/install/mission_control_interfaces/lib/libmission_control_interfaces__rosidl_typesupport_cpp.so"
  "planner_launch=${WORKSPACE_ROOT}/install/navigation_mission_manager/share/navigation_mission_manager/launch/mission_navigation.launch.py"
  "planner_map=${WORKSPACE_ROOT}/install/new_map_planner/share/new_map_planner/config/arena.yaml"
  "vehicle_physical=${WORKSPACE_ROOT}/install/vehicle_config/share/vehicle_config/config/vehicle_physical.yaml"
)
ARTIFACT_ARGUMENTS=()
for artifact in "${PLANNER_ARTIFACTS[@]}"; do
  ARTIFACT_ARGUMENTS+=(--artifact "${artifact}")
done

SOURCE_FINGERPRINT_BEFORE="$(${BUILD_MANIFEST_HELPER} fingerprint --workspace "${WORKSPACE_ROOT}")" ||
  fail "无法计算源码指纹"
if BUILD_ID="$(${BUILD_MANIFEST_HELPER} verify-build \
    --workspace "${WORKSPACE_ROOT}" \
    --manifest "${BUILD_MANIFEST}" \
    "${ARTIFACT_ARGUMENTS[@]}" 2>/dev/null)"; then
  log "构建产物与当前源码一致，跳过构建"
elif [[ "${BUILD_IF_NEEDED}" == "false" ]]; then
  release_workspace_build_lock
  fail "--no-build 验证失败；请去掉 --no-build 重新运行"
else
  log "源码或构建产物已变化，执行增量构建"
  (
    cd "${WORKSPACE_ROOT}"
    "${COLCON_COMMAND}" build --symlink-install --packages-select \
      vehicle_config mission_control_interfaces navigation_safety contrast_navigation_core final_navigation new_map_planner \
      navigation_mission_manager
  )
  SOURCE_FINGERPRINT_AFTER="$(${BUILD_MANIFEST_HELPER} fingerprint --workspace "${WORKSPACE_ROOT}")" ||
    fail "构建后无法重新计算源码指纹"
  [[ "${SOURCE_FINGERPRINT_AFTER}" == "${SOURCE_FINGERPRINT_BEFORE}" ]] ||
    fail "构建期间源码发生变化，请停止编辑后重试"
  BUILD_ID="$(${BUILD_MANIFEST_HELPER} record-build \
    --workspace "${WORKSPACE_ROOT}" \
    --output "${BUILD_MANIFEST}" \
    --expected-fingerprint "${SOURCE_FINGERPRINT_AFTER}" \
    --setting "ros_distro=${ROS_DISTRO_NAME}" \
    "${ARTIFACT_ARGUMENTS[@]}")" ||
    fail "无法记录新规划器构建清单"
fi
release_workspace_build_lock

set +u
# shellcheck disable=SC1090
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

cleanup_previous_planner

if "${ROS2_COMMAND}" service type /new_map_planner/plan >/dev/null 2>&1; then
  fail "已有 /new_map_planner/plan 服务；请先关闭旧的新规划器"
fi
if "${ROS2_COMMAND}" service type /navigation_mission/control >/dev/null 2>&1; then
  fail "已有 /navigation_mission/control 服务；请先关闭旧任务节点"
fi

log "地图: ${MAP_FILE}"
log "模式: 实车自主任务（不启动仿真）"
export PYTHONDONTWRITEBYTECODE=1
exec "${ROS2_COMMAND}" launch navigation_mission_manager mission_navigation.launch.py \
  map_file:="${MAP_FILE}" start_ui:="${START_UI}"
