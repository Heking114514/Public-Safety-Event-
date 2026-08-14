#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_MAP="${WORKSPACE_DIR}/src/arena_path_planner/config/arena_map.yaml"
MAP_FILE="${ARENA_MAP:-${DEFAULT_MAP}}"

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
export ARENA_MAP="${MAP_FILE}"
printf '[arena-planner] Map: %s\n' "${ARENA_MAP}"

# ROS 2 generated setup files read optional variables before defining them,
# which is incompatible with nounset on some Humble installations.
set +u
source /opt/ros/humble/setup.bash
set -u
(cd "${WORKSPACE_DIR}" && colcon build --symlink-install --packages-select arena_path_planner)
set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

BACKEND_PID=""
# A planner process owns its map at startup. Reusing an old service can silently
# mix a previous map with the current frontend, so always start one for this UI.
while read -r planner_pid; do
  [[ -n "${planner_pid}" ]] || continue
  kill "${planner_pid}" 2>/dev/null || true
done < <(pgrep -f '/arena_path_planner/lib/arena_path_planner/arena_planner_node' || true)
ros2 launch arena_path_planner arena_path_planner.launch.py config:="${MAP_FILE}" &
BACKEND_PID=$!
cleanup() {
  if [[ -n "${BACKEND_PID}" ]]; then
    kill "${BACKEND_PID}" 2>/dev/null || true
    wait "${BACKEND_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

for _ in {1..50}; do
  if ros2 service type /arena_path_planner/plan >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

python3 "${SCRIPT_DIR}/arena_route_frontend.py"
