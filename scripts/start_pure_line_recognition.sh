#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${PURE_LINE_ROS_SETUP:-${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}}"
ROS2_COMMAND="${PURE_LINE_ROS2_COMMAND:-ros2}"

log() {
  printf '[pure-line-recognition] %s\n' "$*"
}

fail() {
  printf '[pure-line-recognition] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: scripts/start_pure_line_recognition.sh [launch_arg:=value ...] [record_bag:=true|false]

Starts the D455 camera, the pure path-recognition node, and the live GUI.
It does not start the serial bridge and never publishes drive commands.

Examples:
  scripts/start_pure_line_recognition.sh
  scripts/start_pure_line_recognition.sh viewer_mode:=camera
  scripts/start_pure_line_recognition.sh viewer_mode:=debug
  scripts/start_pure_line_recognition.sh record_bag:=false
  scripts/start_pure_line_recognition.sh start_camera:=false image_topic:=/your/image

Environment:
  PURE_LINE_RECORD_BAG       true or false, default true
  PURE_LINE_RECORDING_ROOT   Recording root that contains path_recognition_runs
  PURE_LINE_RUNS_ROOT        Output root for timestamped recognition runs
  PURE_LINE_LATEST_RUN       Symlink to the newest recognition run
  PURE_LINE_LATEST_BAG       Symlink to the newest recognition bag
  PURE_LINE_ROS2_COMMAND     ros2 command override
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

[[ -f "${ROS_SETUP}" ]] || fail "ROS setup is missing: ${ROS_SETUP}"
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
else
  fail "workspace setup is missing; run colcon build first: ${WORKSPACE_ROOT}/install/setup.bash"
fi

[[ -f "${WORKSPACE_ROOT}/install/vehicle_config/share/vehicle_config/config/vehicle_physical.yaml" ]] ||
  fail "vehicle physical config is missing; rebuild vehicle_config"

has_launch_arg() {
  local key="$1" argument
  shift
  for argument in "$@"; do
    [[ "${argument}" == "${key}:="* ]] && return 0
  done
  return 1
}

get_arg() {
  local key="$1" default_value="$2" argument
  for argument in "${LAUNCH_ARGS[@]}"; do
    if [[ "${argument}" == "${key}:="* ]]; then
      printf '%s\n' "${argument#${key}:=}"
      return 0
    fi
  done
  printf '%s\n' "${default_value}"
}

truthy() {
  case "${1,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

RECORD_BAG="${PURE_LINE_RECORD_BAG:-true}"
LAUNCH_ARGS=()
for argument in "$@"; do
  case "${argument}" in
    record_bag:=*) RECORD_BAG="${argument#record_bag:=}" ;;
    *) LAUNCH_ARGS+=("${argument}") ;;
  esac
done

if ! has_launch_arg "show_debug_gui" "${LAUNCH_ARGS[@]}"; then
  LAUNCH_ARGS+=("show_debug_gui:=true")
fi
if ! has_launch_arg "publish_debug_image" "${LAUNCH_ARGS[@]}"; then
  LAUNCH_ARGS+=("publish_debug_image:=true")
fi
if ! has_launch_arg "viewer_mode" "${LAUNCH_ARGS[@]}"; then
  LAUNCH_ARGS+=("viewer_mode:=side_by_side")
fi

IMAGE_TOPIC="$(get_arg image_topic /camera/camera/color/image_raw)"
CAMERA_INFO_TOPIC="$(get_arg camera_info_topic /camera/camera/color/camera_info)"
DEPTH_IMAGE_TOPIC="$(get_arg depth_image_topic /camera/camera/aligned_depth_to_color/image_raw)"
STATUS_TOPIC="$(get_arg status_topic /pure_line_recognition/status)"
DEBUG_IMAGE_TOPIC="$(get_arg debug_image_topic /pure_line_recognition/debug_image)"

find_recording_root() {
  if [[ -n "${PURE_LINE_RECORDING_ROOT:-}" ]]; then
    mkdir -p -- "${PURE_LINE_RECORDING_ROOT}" 2>/dev/null || return 1
    [[ -d "${PURE_LINE_RECORDING_ROOT}" && -w "${PURE_LINE_RECORDING_ROOT}" ]] &&
      printf '%s\n' "${PURE_LINE_RECORDING_ROOT}" &&
      return 0
  fi
  local candidate parent
  for candidate in /media/hjh/E/rosbag_recording /media/hjh/Data/rosbag_recording; do
    parent="$(dirname -- "${candidate}")"
    [[ -d "${parent}" ]] || continue
    mkdir -p -- "${candidate}" 2>/dev/null || true
    if [[ -d "${candidate}" && -w "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

RUN_DIR=""
BAG_DIR=""
LATEST_RUN=""
LATEST_BAG=""
BAG_PID=""
LAUNCH_PID=""
CLEANED_UP="false"

if truthy "${RECORD_BAG}"; then
  RECORDING_ROOT="$(find_recording_root || true)"
  if [[ -n "${PURE_LINE_RUNS_ROOT:-}" ]]; then
    RUNS_ROOT="${PURE_LINE_RUNS_ROOT}"
    RUNS_PARENT="$(dirname -- "${RUNS_ROOT}")"
    LATEST_RUN="${PURE_LINE_LATEST_RUN:-${RUNS_PARENT}/latest_path_recognition_run}"
    LATEST_BAG="${PURE_LINE_LATEST_BAG:-${RUNS_PARENT}/latest_path_recognition_bag}"
  elif [[ -n "${RECORDING_ROOT}" ]]; then
    RUNS_ROOT="${RECORDING_ROOT}/path_recognition_runs"
    LATEST_RUN="${PURE_LINE_LATEST_RUN:-${RECORDING_ROOT}/latest_path_recognition_run}"
    LATEST_BAG="${PURE_LINE_LATEST_BAG:-${RECORDING_ROOT}/latest_path_recognition_bag}"
  else
    RUNS_ROOT=""
    log "no writable recording root found; live recognition will run without rosbag"
  fi
  if [[ -n "${RUNS_ROOT:-}" ]]; then
    SOURCE_SHORT_COMMIT="$(git -C "${WORKSPACE_ROOT}" rev-parse --short=12 HEAD 2>/dev/null || printf unknown)"
    RUN_ID="$(date -u +%Y%m%dT%H%M%S.%6NZ)_${SOURCE_SHORT_COMMIT}_path_recognition"
    RUN_DIR="${RUNS_ROOT}/${RUN_ID}"
    BAG_DIR="${RUN_DIR}/bag"
    mkdir -p -- "${RUN_DIR}"
  fi
fi

cleanup() {
  local original_status=$?
  local pid
  [[ "${CLEANED_UP}" == "false" ]] || return 0
  CLEANED_UP="true"
  trap - EXIT INT TERM
  set +e
  for pid in "${BAG_PID}" "${LAUNCH_PID}"; do
    [[ -n "${pid}" ]] || continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${BAG_PID}" "${LAUNCH_PID}"; do
    [[ -n "${pid}" ]] || continue
    wait "${pid}" 2>/dev/null || true
  done
  if [[ -n "${BAG_DIR}" && -f "${BAG_DIR}/metadata.yaml" ]]; then
    mkdir -p -- "$(dirname -- "${LATEST_BAG}")" "$(dirname -- "${LATEST_RUN}")"
    ln -sfnT -- "${BAG_DIR}" "${LATEST_BAG}" 2>/dev/null || true
    ln -sfnT -- "${RUN_DIR}" "${LATEST_RUN}" 2>/dev/null || true
    log "Latest recognition bag link: ${LATEST_BAG}"
  fi
  exit "${original_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

log "Starting path recognition only: no serial bridge, no drive commands"
log "Camera topic: ${IMAGE_TOPIC}"
log "Status topic: ${STATUS_TOPIC}"
log "Debug image topic: ${DEBUG_IMAGE_TOPIC}"
if [[ -n "${BAG_DIR}" ]]; then
  log "Recording bag: ${BAG_DIR}"
fi

"${ROS2_COMMAND}" launch pure_line_following pure_line_recognition.launch.py \
  "${LAUNCH_ARGS[@]}" &
LAUNCH_PID="$!"

if [[ -n "${BAG_DIR}" ]]; then
  sleep 0.7
  "${ROS2_COMMAND}" bag record --output "${BAG_DIR}" \
    --max-bag-size "${PURE_LINE_ROSBAG_MAX_BAG_SIZE:-1073741824}" \
    --compression-mode "${PURE_LINE_ROSBAG_COMPRESSION_MODE:-file}" \
    --compression-format "${PURE_LINE_ROSBAG_COMPRESSION_FORMAT:-zstd}" \
    "${IMAGE_TOPIC}" \
    "${CAMERA_INFO_TOPIC}" \
    "${DEPTH_IMAGE_TOPIC}" \
    /camera/camera/aligned_depth_to_color/camera_info \
    "${STATUS_TOPIC}" \
    "${DEBUG_IMAGE_TOPIC}" \
    /parameter_events \
    /rosout &
  BAG_PID="$!"
  wait -n "${LAUNCH_PID}" "${BAG_PID}"
else
  wait "${LAUNCH_PID}"
fi
