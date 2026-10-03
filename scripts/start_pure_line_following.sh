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
  scripts/start_pure_line_following.sh show_debug_gui:=true
  scripts/start_pure_line_following.sh show_debug_gui:=true viewer_mode:=camera
  scripts/start_pure_line_following.sh start_camera:=false publish_debug_image:=false

Environment:
  PURE_LINE_RUNS_ROOT       Output root for timestamped runs
  PURE_LINE_LATEST_RUN      Symlink to the newest run
  PURE_LINE_LATEST_BAG      Symlink to the newest bag
  PURE_LINE_RECORDING_ROOT  Recording root that contains navigation_runs
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

[[ -f "${WORKSPACE_ROOT}/install/vehicle_config/share/vehicle_config/config/vehicle_physical.yaml" ]] ||
  fail "vehicle physical config is missing; rebuild vehicle_config"

# Keep pure-line bags in the same external tree as navigation bags. Do not fall
# back to the workspace root: local bags get mixed into source control and are
# easy to delete or analyze from the wrong place.
mount_recording_volume() {
  local label="$1" mountpoint="$2" device="/dev/disk/by-label/$1"
  [[ -e "${device}" ]] || return 1
  if command -v udisksctl >/dev/null 2>&1; then
    udisksctl mount -b "${device}" >/dev/null 2>&1 || true
    mountpoint -q -- "${mountpoint}" && return 0
  fi
  if [[ ! -d "${mountpoint}" ]]; then
    mkdir -p -- "${mountpoint}" 2>/dev/null \
      || sudo mkdir -p -- "${mountpoint}" >/dev/null 2>&1 \
      || return 1
  fi
  sudo mount -t ntfs3 -o force "${device}" "${mountpoint}" >/dev/null 2>&1 || true
  mountpoint -q -- "${mountpoint}" && return 0
  sudo ntfsfix "${device}" >/dev/null 2>&1 || true
  sudo mount -t ntfs3 "${device}" "${mountpoint}" >/dev/null 2>&1 || true
  mountpoint -q -- "${mountpoint}"
}

DEFAULT_RECORDING_ROOT="${PURE_LINE_RECORDING_ROOT:-}"
if [[ -n "${DEFAULT_RECORDING_ROOT}" ]]; then
  mkdir -p -- "${DEFAULT_RECORDING_ROOT}" 2>/dev/null ||
    fail "recording root is not writable: ${DEFAULT_RECORDING_ROOT}"
  [[ -d "${DEFAULT_RECORDING_ROOT}" && -w "${DEFAULT_RECORDING_ROOT}" ]] ||
    fail "recording root is not writable: ${DEFAULT_RECORDING_ROOT}"
fi

for candidate in /media/hjh/E/rosbag_recording /media/hjh/Data/rosbag_recording; do
  [[ -z "${DEFAULT_RECORDING_ROOT}" ]] || break
  candidate_parent="$(dirname -- "${candidate}")"
  if [[ -d "${candidate_parent}" ]]; then
    mkdir -p -- "${candidate}" 2>/dev/null || true
    if [[ -d "${candidate}" && -w "${candidate}" ]]; then
      DEFAULT_RECORDING_ROOT="${candidate}"
      break
    fi
  fi
  if [[ -z "${PURE_LINE_RUNS_ROOT:-}" ]] \
    && mount_recording_volume "$(basename -- "${candidate_parent}")" "${candidate_parent}" \
    && mountpoint -q -- "${candidate_parent}"; then
    mkdir -p -- "${candidate}" 2>/dev/null || true
    if [[ -d "${candidate}" && -w "${candidate}" ]]; then
      log "mounted ${candidate_parent} for recording"
      DEFAULT_RECORDING_ROOT="${candidate}"
      break
    fi
  fi
done

if [[ -n "${DEFAULT_RECORDING_ROOT}" ]]; then
  RUNS_ROOT="${PURE_LINE_RUNS_ROOT:-${DEFAULT_RECORDING_ROOT}/navigation_runs}"
  LATEST_RUN="${PURE_LINE_LATEST_RUN:-${DEFAULT_RECORDING_ROOT}/latest_navigation_run}"
  LATEST_BAG="${PURE_LINE_LATEST_BAG:-${DEFAULT_RECORDING_ROOT}/latest_navigation_bag}"
else
  [[ -n "${PURE_LINE_RUNS_ROOT:-}" ]] ||
    fail "no writable external rosbag directory found; mount /media/hjh/E or /media/hjh/Data writable, or set PURE_LINE_RUNS_ROOT"
  RUNS_ROOT="${PURE_LINE_RUNS_ROOT}"
  RUNS_PARENT="$(dirname -- "${RUNS_ROOT}")"
  LATEST_RUN="${PURE_LINE_LATEST_RUN:-${RUNS_PARENT}/latest_navigation_run}"
  LATEST_BAG="${PURE_LINE_LATEST_BAG:-${RUNS_PARENT}/latest_navigation_bag}"
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

SOURCE_SHORT_COMMIT="$(git -C "${WORKSPACE_ROOT}" rev-parse --short=12 HEAD 2>/dev/null || printf unknown)"
RUN_ID="$(date -u +%Y%m%dT%H%M%S.%6NZ)_${SOURCE_SHORT_COMMIT}_pure_line"
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
  local original_status=$?
  local pid
  local attempt
  local -a remaining=()
  [[ "${CLEANED_UP}" == "false" ]] || return 0
  CLEANED_UP="true"
  trap - EXIT INT TERM
  set +e
  for pid in "${BAG_PID}" "${LAUNCH_PID}"; do
    [[ -n "${pid}" ]] || continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  for attempt in {1..80}; do
    remaining=()
    for pid in "${BAG_PID}" "${LAUNCH_PID}"; do
      [[ -n "${pid}" ]] || continue
      kill -0 "${pid}" 2>/dev/null && remaining+=("${pid}")
    done
    ((${#remaining[@]} == 0)) && break
    sleep 0.1
  done
  if ((${#remaining[@]} > 0)); then
    kill -TERM "${remaining[@]}" 2>/dev/null || true
    for attempt in {1..10}; do
      local -a still_running=()
      for pid in "${remaining[@]}"; do
        kill -0 "${pid}" 2>/dev/null && still_running+=("${pid}")
      done
      remaining=("${still_running[@]}")
      ((${#remaining[@]} == 0)) && break
      sleep 0.1
    done
  fi
  ((${#remaining[@]} == 0)) || kill -KILL "${remaining[@]}" 2>/dev/null || true
  for pid in "${BAG_PID}" "${LAUNCH_PID}"; do
    [[ -n "${pid}" ]] || continue
    wait "${pid}" 2>/dev/null || true
  done
  if [[ "${original_status}" == "0" || "${original_status}" == "130" || "${original_status}" == "143" ]]; then
    if [[ -f "${BAG_DIR}/metadata.yaml" ]]; then
      mkdir -p -- "$(dirname -- "${LATEST_BAG}")"
      if ln -sfnT -- "${BAG_DIR}" "${LATEST_BAG}"; then
        log "Latest navigation bag link: ${LATEST_BAG}"
      else
        log "could not update latest bag link; leaving existing link untouched: ${LATEST_BAG}"
      fi
    else
      log "bag metadata missing; latest navigation bag link was not updated"
    fi
  else
    log "run failed with status ${original_status}; latest navigation bag link was not updated"
  fi
  exit "${original_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

log "Run directory: ${RUN_DIR}"
log "Recording bag: ${BAG_DIR}"
log "Latest run link: ${LATEST_RUN}"
log "Command topic: ${CMD_VEL_TOPIC}"
ln -sfnT -- "${RUN_DIR}" "${LATEST_RUN}"

"${ROS2_COMMAND}" launch pure_line_following pure_line_following_serial.launch.py \
  "${LAUNCH_ARGS[@]}" &
LAUNCH_PID="$!"

sleep 0.7

"${ROS2_COMMAND}" bag record --output "${BAG_DIR}" \
  "${ROSBAG_OPTIONS[@]}" \
  "${ROSBAG_TOPICS[@]}" &
BAG_PID="$!"

set +e
wait -n "${LAUNCH_PID}" "${BAG_PID}"
STATUS="$?"
set -e

log "A process exited with status ${STATUS}; stopping pure line following"
exit "${STATUS}"
