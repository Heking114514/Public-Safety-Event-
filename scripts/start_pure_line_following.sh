#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${PURE_LINE_ROS_SETUP:-${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}}"
ROS2_COMMAND="${PURE_LINE_ROS2_COMMAND:-ros2}"

log() {
  printf '[pure-line] %s\n' "$*"
}

fail() {
  printf '[pure-line] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: scripts/start_pure_line_following.sh [launch_arg:=value ...]

Starts the RGB pure line follower, the serial bridge, and a rosbag recording.
Arguments are passed through to pure_line_following_serial.launch.py.

Examples:
  scripts/start_pure_line_following.sh
  scripts/start_pure_line_following.sh serial_device:=/dev/serial/by-id/your-controller
  scripts/start_pure_line_following.sh start_camera:=false publish_debug_image:=false

Environment:
  PURE_LINE_RUNS_ROOT       Output root for timestamped runs
  PURE_LINE_LATEST_RUN      Symlink to the newest run
  PURE_LINE_LATEST_BAG      Symlink to the newest bag
  PURE_LINE_ROS2_COMMAND    ros2 command override
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

DEFAULT_RECORDING_ROOT=""
for candidate in \
  /media/hjh/E/rosbag_recording \
  /media/hjh/Data/rosbag_recording \
  /media/hjh/E1/rosbag_recording; do
  candidate_parent="$(dirname -- "${candidate}")"
  if [[ -d "${candidate_parent}" ]]; then
    mkdir -p -- "${candidate}" 2>/dev/null || true
    if [[ -d "${candidate}" && -w "${candidate}" ]]; then
      DEFAULT_RECORDING_ROOT="${candidate}"
      break
    fi
  fi
done

if [[ -n "${DEFAULT_RECORDING_ROOT}" ]]; then
  RUNS_ROOT="${PURE_LINE_RUNS_ROOT:-${DEFAULT_RECORDING_ROOT}/pure_line_runs}"
  LATEST_RUN="${PURE_LINE_LATEST_RUN:-${DEFAULT_RECORDING_ROOT}/latest_pure_line_run}"
  LATEST_BAG="${PURE_LINE_LATEST_BAG:-${DEFAULT_RECORDING_ROOT}/latest_pure_line_bag}"
else
  RUNS_ROOT="${PURE_LINE_RUNS_ROOT:-${WORKSPACE_ROOT}/pure_line_runs}"
  LATEST_RUN="${PURE_LINE_LATEST_RUN:-${WORKSPACE_ROOT}/latest_pure_line_run}"
  LATEST_BAG="${PURE_LINE_LATEST_BAG:-${WORKSPACE_ROOT}/latest_pure_line_bag}"
  log "external rosbag directory not found; recording under ${RUNS_ROOT}"
fi

has_launch_arg() {
  local key="$1" argument
  shift
  for argument in "$@"; do
    [[ "${argument}" == "${key}:="* ]] && return 0
  done
  return 1
}

LAUNCH_ARGS=("$@")
if ! has_launch_arg "publish_debug_image" "${LAUNCH_ARGS[@]}"; then
  LAUNCH_ARGS+=("publish_debug_image:=true")
fi

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

IMAGE_TOPIC="$(get_arg image_topic /camera/camera/color/image_raw)"
CAMERA_INFO_TOPIC="$(get_arg camera_info_topic /camera/camera/color/camera_info)"
DEPTH_IMAGE_TOPIC="$(get_arg depth_image_topic /camera/camera/aligned_depth_to_color/image_raw)"
CMD_VEL_TOPIC="$(get_arg cmd_vel_topic /cmd_vel_line_following)"
DEBUG_IMAGE_TOPIC="$(get_arg debug_image_topic /pure_line_following/debug_image)"

RUN_ID="$(date -u +%Y%m%dT%H%M%S.%6NZ)_pure_line"
RUN_DIR="${RUNS_ROOT}/${RUN_ID}"
BAG_DIR="${RUN_DIR}/bag"
mkdir -p -- "${RUN_DIR}"

declare -a ROSBAG_OPTIONS=(
  --max-bag-size "${PURE_LINE_ROSBAG_MAX_BAG_SIZE:-1073741824}"
  --compression-mode "${PURE_LINE_ROSBAG_COMPRESSION_MODE:-file}"
  --compression-format "${PURE_LINE_ROSBAG_COMPRESSION_FORMAT:-zstd}"
)

declare -a RAW_TOPICS=(
  "${IMAGE_TOPIC}"
  "${CAMERA_INFO_TOPIC}"
  "${DEPTH_IMAGE_TOPIC}"
  /camera/camera/aligned_depth_to_color/camera_info
  /pure_line_following/status
  "${CMD_VEL_TOPIC}"
  "${DEBUG_IMAGE_TOPIC}"
  /cup_car_serial/control_telemetry
  /cup_car_serial/connected
  /cup_car_serial/actuator_healthy
  /cup_car_serial/actuator_tracking_status
  /cup_car_serial/rx
  /parameter_events
  /rosout
)
declare -A SEEN_TOPICS=()
declare -a ROSBAG_TOPICS=()
for topic in "${RAW_TOPICS[@]}"; do
  [[ -n "${topic}" ]] || continue
  if [[ -z "${SEEN_TOPICS[${topic}]:-}" ]]; then
    SEEN_TOPICS["${topic}"]=1
    ROSBAG_TOPICS+=("${topic}")
  fi
done

LAUNCH_PID=""
BAG_PID=""
CLEANED_UP="false"

cleanup() {
  local pid
  [[ "${CLEANED_UP}" == "false" ]] || return 0
  CLEANED_UP="true"
  set +e
  for pid in "${BAG_PID}" "${LAUNCH_PID}"; do
    [[ -n "${pid}" ]] || continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${BAG_PID}" "${LAUNCH_PID}"; do
    [[ -n "${pid}" ]] || continue
    wait "${pid}" 2>/dev/null || true
  done
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

log "Run directory: ${RUN_DIR}"
log "Recording bag: ${BAG_DIR}"
log "Command topic: ${CMD_VEL_TOPIC}"

"${ROS2_COMMAND}" launch pure_line_following pure_line_following_serial.launch.py \
  "${LAUNCH_ARGS[@]}" &
LAUNCH_PID="$!"

sleep 0.7

"${ROS2_COMMAND}" bag record --output "${BAG_DIR}" \
  "${ROSBAG_OPTIONS[@]}" \
  "${ROSBAG_TOPICS[@]}" &
BAG_PID="$!"

ln -sfn -- "${RUN_DIR}" "${LATEST_RUN}"
ln -sfn -- "${BAG_DIR}" "${LATEST_BAG}"
log "Latest bag link: ${LATEST_BAG}"

set +e
wait -n "${LAUNCH_PID}" "${BAG_PID}"
STATUS="$?"
set -e

log "A process exited with status ${STATUS}; stopping pure line following"
exit "${STATUS}"
