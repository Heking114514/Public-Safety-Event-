#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="${CAMERA_SAMPLE_ROS_SETUP:-${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}}"
ROS2_COMMAND="${CAMERA_SAMPLE_ROS2_COMMAND:-ros2}"
PGREP_COMMAND="${CAMERA_SAMPLE_PGREP_COMMAND:-pgrep}"

log() {
  printf '[camera-samples] %s\n' "$*"
}

fail() {
  printf '[camera-samples] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: scripts/record_camera_samples.sh [options]

Starts only the D455 camera. After placing the stationary car at a sample
position, enter an optional note and press Enter to record one short rosbag.
Move the car between samples and enter q when finished. No serial bridge or
motion-control node is started.

Options:
  --duration SECONDS       Seconds per sample (positive integer, default: 8)
  --samples COUNT          Stop after COUNT samples (default: unlimited)
  --camera-serial SERIAL   RealSense serial (default: detect the first device)
  --color-profile PROFILE  Color profile (default: 640x480x15)
  --depth-profile PROFILE  Depth profile (default: 640x480x15)
  --no-depth               Record RGB and calibration only
  --no-camera              Use an already-running camera node
  --output-root DIR        Store sessions below DIR instead of an external disk
  -h, --help               Show this help

Examples:
  scripts/record_camera_samples.sh
  scripts/record_camera_samples.sh --duration 5 --samples 12
  scripts/record_camera_samples.sh --no-depth
  scripts/record_camera_samples.sh --no-camera --output-root /data/camera_samples

Environment:
  CAMERA_SAMPLE_OUTPUT_ROOT       Session output root
  CAMERA_SAMPLE_ROS2_COMMAND      ros2 command override
  CAMERA_SAMPLE_ROS_SETUP         ROS setup script override
  CAMERA_SAMPLE_WARMUP_S          Camera warm-up seconds (default: 2)
  CAMERA_SAMPLE_TOPIC_TIMEOUT_S   Topic wait timeout (default: 20)
EOF
}

DURATION_S="${CAMERA_SAMPLE_DURATION_S:-8}"
MAX_SAMPLES="${CAMERA_SAMPLE_COUNT:-0}"
CAMERA_SERIAL="${CAMERA_SAMPLE_CAMERA_SERIAL:-}"
COLOR_PROFILE="${CAMERA_SAMPLE_COLOR_PROFILE:-640x480x15}"
DEPTH_PROFILE="${CAMERA_SAMPLE_DEPTH_PROFILE:-640x480x15}"
START_CAMERA="true"
ENABLE_DEPTH="true"
OUTPUT_ROOT="${CAMERA_SAMPLE_OUTPUT_ROOT:-}"
WARMUP_S="${CAMERA_SAMPLE_WARMUP_S:-2}"
TOPIC_TIMEOUT_S="${CAMERA_SAMPLE_TOPIC_TIMEOUT_S:-20}"

while (($# > 0)); do
  case "$1" in
    --duration)
      (($# >= 2)) || fail "--duration requires a positive integer"
      DURATION_S="$2"
      shift 2
      ;;
    --samples)
      (($# >= 2)) || fail "--samples requires a positive integer"
      MAX_SAMPLES="$2"
      shift 2
      ;;
    --camera-serial)
      (($# >= 2)) || fail "--camera-serial requires a serial number"
      CAMERA_SERIAL="$2"
      shift 2
      ;;
    --color-profile)
      (($# >= 2)) || fail "--color-profile requires a profile"
      COLOR_PROFILE="$2"
      shift 2
      ;;
    --depth-profile)
      (($# >= 2)) || fail "--depth-profile requires a profile"
      DEPTH_PROFILE="$2"
      shift 2
      ;;
    --no-depth)
      ENABLE_DEPTH="false"
      shift
      ;;
    --no-camera)
      START_CAMERA="false"
      shift
      ;;
    --output-root)
      (($# >= 2)) || fail "--output-root requires a directory"
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown option '$1' (use --help)"
      ;;
  esac
done

[[ "${DURATION_S}" =~ ^[1-9][0-9]*$ ]] ||
  fail "--duration must be a positive integer"
[[ "${MAX_SAMPLES}" =~ ^[0-9]+$ ]] ||
  fail "--samples must be a non-negative integer (0 means unlimited)"
[[ "${WARMUP_S}" =~ ^[0-9]+$ ]] ||
  fail "CAMERA_SAMPLE_WARMUP_S must be a non-negative integer"
[[ "${TOPIC_TIMEOUT_S}" =~ ^[1-9][0-9]*$ ]] ||
  fail "CAMERA_SAMPLE_TOPIC_TIMEOUT_S must be a positive integer"

[[ -f "${ROS_SETUP}" ]] || fail "ROS setup is missing: ${ROS_SETUP}"
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
command -v "${ROS2_COMMAND}" >/dev/null 2>&1 ||
  fail "ros2 command is unavailable: ${ROS2_COMMAND}"

mount_recording_volume() {
  local label="$1"
  local mount_path="$2"
  local device="/dev/disk/by-label/${label}"

  mountpoint -q -- "${mount_path}" 2>/dev/null && return 0
  [[ -e "${device}" ]] || return 1

  # udisks chooses the mount directory from the volume label. Do not create
  # that directory first, otherwise it may silently mount as E1/Data1.
  if [[ ! -e "${mount_path}" ]] && command -v udisksctl >/dev/null 2>&1; then
    udisksctl mount -b "${device}" >/dev/null 2>&1 || true
    mountpoint -q -- "${mount_path}" 2>/dev/null && return 0
  fi

  if [[ ! -d "${mount_path}" ]]; then
    mkdir -p -- "${mount_path}" 2>/dev/null ||
      sudo mkdir -p -- "${mount_path}" >/dev/null 2>&1 ||
      return 1
  fi
  sudo mount -t ntfs3 -o force "${device}" "${mount_path}" \
    >/dev/null 2>&1 || true
  mountpoint -q -- "${mount_path}" 2>/dev/null && return 0

  sudo ntfsfix "${device}" >/dev/null 2>&1 || true
  sudo mount -t ntfs3 "${device}" "${mount_path}" >/dev/null 2>&1 || true
  mountpoint -q -- "${mount_path}" 2>/dev/null
}

find_output_root() {
  local mount_path
  local candidate

  if [[ -n "${OUTPUT_ROOT}" ]]; then
    mkdir -p -- "${OUTPUT_ROOT}" || return 1
    [[ -d "${OUTPUT_ROOT}" && -w "${OUTPUT_ROOT}" ]] || return 1
    printf '%s\n' "${OUTPUT_ROOT}"
    return 0
  fi

  for mount_path in /media/hjh/E /media/hjh/Data; do
    if ! mountpoint -q -- "${mount_path}" 2>/dev/null; then
      mount_recording_volume "$(basename -- "${mount_path}")" "${mount_path}" ||
        continue
    fi
    candidate="${mount_path}/rosbag_recording/camera_sample_runs"
    mkdir -p -- "${candidate}" 2>/dev/null || continue
    if [[ -d "${candidate}" && -w "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

OUTPUT_ROOT="$(find_output_root || true)"
[[ -n "${OUTPUT_ROOT}" ]] ||
  fail "no writable external recording disk found; mount /media/hjh/E or /media/hjh/Data, or pass --output-root"

if [[ "${START_CAMERA}" == "true" ]]; then
  command -v rs-enumerate-devices >/dev/null 2>&1 ||
    fail "rs-enumerate-devices is unavailable; install ros-${ROS_DISTRO_NAME}-realsense2-camera"
  if "${PGREP_COMMAND}" -f '/realsense2_camera_node([[:space:]]|$)' >/dev/null 2>&1; then
    fail "a RealSense camera node is already running; use --no-camera or stop it first"
  fi

  DEVICE_INFO="$(rs-enumerate-devices 2>&1)" ||
    fail "RealSense device query failed: ${DEVICE_INFO}"
  grep -q "No device detected" <<<"${DEVICE_INFO}" &&
    fail "D455 not detected; reconnect it to a USB 3.x port"
  if [[ -z "${CAMERA_SERIAL}" ]]; then
    CAMERA_SERIAL="$(awk -F: '/^[[:space:]]*Serial Number[[:space:]]*:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${DEVICE_INFO}")"
  fi
  [[ -n "${CAMERA_SERIAL}" ]] || fail "could not read the D455 serial number"
  CAMERA_SERIAL="${CAMERA_SERIAL#_}"
else
  CAMERA_SERIAL="${CAMERA_SERIAL#_}"
  [[ -n "${CAMERA_SERIAL}" ]] || CAMERA_SERIAL="external"
fi

SOURCE_SHORT_COMMIT="$(git -C "${WORKSPACE_ROOT}" rev-parse --short=12 HEAD 2>/dev/null || printf unknown)"
SESSION_ID="$(date -u +%Y%m%dT%H%M%S.%6NZ)_${SOURCE_SHORT_COMMIT}_camera_samples"
SESSION_DIR="${OUTPUT_ROOT}/${SESSION_ID}"
SAMPLES_DIR="${SESSION_DIR}/samples"
INDEX_FILE="${SESSION_DIR}/samples.tsv"
CAMERA_LOG="${SESSION_DIR}/camera.log"
mkdir -p -- "${SAMPLES_DIR}"
printf 'sample_id\tnote\tstarted_at_utc\tduration_s\tstatus\tbag\n' >"${INDEX_FILE}"

COLOR_TOPIC="/camera/camera/color/image_raw"
COLOR_INFO_TOPIC="/camera/camera/color/camera_info"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
DEPTH_INFO_TOPIC="/camera/camera/aligned_depth_to_color/camera_info"

declare -a BAG_TOPICS=(
  "${COLOR_TOPIC}"
  "${COLOR_INFO_TOPIC}"
)
if [[ "${ENABLE_DEPTH}" == "true" ]]; then
  BAG_TOPICS+=("${DEPTH_TOPIC}" "${DEPTH_INFO_TOPIC}")
fi
BAG_TOPICS+=(/parameter_events /rosout)

CAMERA_PID=""
BAG_PID=""
CLEANED_UP="false"

process_is_running() {
  local pid="$1"
  local state
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/stat" ]] || return 1
  state="$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null)" || return 1
  [[ -n "${state}" && "${state}" != "Z" ]]
}

stop_process() {
  local pid="$1"
  local attempt
  [[ -n "${pid}" ]] || return 0
  process_is_running "${pid}" || {
    wait "${pid}" 2>/dev/null || true
    return 0
  }
  kill -INT "${pid}" 2>/dev/null || true
  for attempt in {1..50}; do
    process_is_running "${pid}" || break
    sleep 0.1
  done
  if process_is_running "${pid}"; then
    kill -TERM "${pid}" 2>/dev/null || true
    for attempt in {1..20}; do
      process_is_running "${pid}" || break
      sleep 0.1
    done
  fi
  process_is_running "${pid}" && kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local original_status=$?
  [[ "${CLEANED_UP}" == "false" ]] || return 0
  CLEANED_UP="true"
  trap - EXIT INT TERM
  set +e
  stop_process "${BAG_PID}"
  BAG_PID=""
  stop_process "${CAMERA_PID}"
  CAMERA_PID=""
  log "Session directory: ${SESSION_DIR}"
  exit "${original_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "${START_CAMERA}" == "true" ]]; then
  log "Starting D455 ${CAMERA_SERIAL}; camera output: ${CAMERA_LOG}"
  "${ROS2_COMMAND}" run realsense2_camera realsense2_camera_node \
    --ros-args \
    -r __ns:=/camera \
    -r __node:=camera \
    -p serial_no:="_${CAMERA_SERIAL}" \
    -p enable_color:=true \
    -p rgb_camera.color_profile:="${COLOR_PROFILE}" \
    -p rgb_camera.color_format:=RGB8 \
    -p enable_depth:="${ENABLE_DEPTH}" \
    -p depth_module.depth_profile:="${DEPTH_PROFILE}" \
    -p depth_module.depth_format:=Z16 \
    -p align_depth.enable:="${ENABLE_DEPTH}" \
    -p enable_infra1:=false \
    -p enable_infra2:=false \
    -p enable_gyro:=false \
    -p enable_accel:=false \
    -p enable_sync:=true \
    -p publish_tf:=false \
    >"${CAMERA_LOG}" 2>&1 &
  CAMERA_PID="$!"
else
  log "Using the existing camera node"
fi

wait_for_camera_topics() {
  local deadline=$((SECONDS + TOPIC_TIMEOUT_S))
  local topics

  while ((SECONDS < deadline)); do
    if [[ -n "${CAMERA_PID}" ]] && ! process_is_running "${CAMERA_PID}"; then
      tail -n 20 "${CAMERA_LOG}" >&2 || true
      fail "camera node exited before publishing images"
    fi
    topics="$("${ROS2_COMMAND}" topic list 2>/dev/null || true)"
    if grep -Fxq "${COLOR_TOPIC}" <<<"${topics}" &&
       grep -Fxq "${COLOR_INFO_TOPIC}" <<<"${topics}"; then
      if [[ "${ENABLE_DEPTH}" != "true" ]] ||
         grep -Fxq "${DEPTH_TOPIC}" <<<"${topics}"; then
        return 0
      fi
    fi
    sleep 0.25
  done
  fail "camera topics were not ready within ${TOPIC_TIMEOUT_S}s"
}

wait_for_camera_topics
if ((WARMUP_S > 0)); then
  log "Camera ready; waiting ${WARMUP_S}s for exposure to settle"
  sleep "${WARMUP_S}"
fi

printf 'camera_serial\t%s\ncolor_profile\t%s\ndepth_enabled\t%s\ndepth_profile\t%s\nduration_s\t%s\n' \
  "${CAMERA_SERIAL}" "${COLOR_PROFILE}" "${ENABLE_DEPTH}" \
  "${DEPTH_PROFILE}" "${DURATION_S}" >"${SESSION_DIR}/session.tsv"

topic_message_count() {
  local metadata_file="$1"
  local expected_topic="$2"

  awk -v expected_topic="${expected_topic}" '
    $1 == "name:" {
      current_topic = $2
      gsub(/^"|"$/, "", current_topic)
    }
    $1 == "message_count:" && current_topic == expected_topic {
      print $2
      found = 1
      exit
    }
    END {
      if (!found) {
        exit 1
      }
    }
  ' "${metadata_file}"
}

print_recording_diagnostics() {
  local bag_dir="$1"

  printf '[camera-samples] ===== ros2 bag info (recording anomaly) =====\n' >&2
  printf '[camera-samples] Bag: %s\n' "${bag_dir}" >&2
  if [[ -f "${bag_dir}/metadata.yaml" ]]; then
    "${ROS2_COMMAND}" bag info "${bag_dir}" >&2 ||
      printf '[camera-samples] ERROR: ros2 bag info failed for %s\n' "${bag_dir}" >&2
  else
    printf '[camera-samples] metadata.yaml is missing: %s\n' "${bag_dir}" >&2
  fi
  if [[ -s "${CAMERA_LOG}" ]]; then
    printf '[camera-samples] ===== camera.log (last 40 lines) =====\n' >&2
    tail -n 40 "${CAMERA_LOG}" >&2 || true
  fi
  printf '[camera-samples] ===== end recording diagnostics =====\n' >&2
}

record_sample() {
  local sample_number="$1"
  local note="$2"
  local sample_id
  local sample_dir
  local bag_dir
  local started_at
  local bag_status=0
  local elapsed=0
  local final_status="complete"
  local failure_reason=""
  local color_message_count=""
  local depth_message_count=""

  printf -v sample_id '%04d' "${sample_number}"
  sample_dir="${SAMPLES_DIR}/${sample_id}"
  bag_dir="${sample_dir}/bag"
  started_at="$(date -u +%Y-%m-%dT%H:%M:%S.%6NZ)"
  mkdir -p -- "${sample_dir}"
  printf '%s\n' "${note}" >"${sample_dir}/note.txt"

  log "Recording sample ${sample_id} for ${DURATION_S}s${note:+ (${note})}"
  "${ROS2_COMMAND}" bag record \
    --output "${bag_dir}" \
    --max-bag-size "${CAMERA_SAMPLE_MAX_BAG_SIZE:-1073741824}" \
    --compression-mode "${CAMERA_SAMPLE_COMPRESSION_MODE:-file}" \
    --compression-format "${CAMERA_SAMPLE_COMPRESSION_FORMAT:-zstd}" \
    "${BAG_TOPICS[@]}" &
  BAG_PID="$!"

  while ((elapsed < DURATION_S)); do
    if ! process_is_running "${BAG_PID}"; then
      break
    fi
    if [[ -n "${CAMERA_PID}" ]] && ! process_is_running "${CAMERA_PID}"; then
      failure_reason="camera node exited during recording"
      break
    fi
    sleep 1
    ((elapsed += 1))
  done

  if process_is_running "${BAG_PID}"; then
    stop_process "${BAG_PID}"
  else
    set +e
    wait "${BAG_PID}"
    bag_status=$?
    set -e
  fi
  BAG_PID=""

  if ((bag_status != 0)); then
    failure_reason="rosbag process exited with status ${bag_status}"
  elif [[ ! -f "${bag_dir}/metadata.yaml" ]]; then
    failure_reason="metadata.yaml was not produced"
  else
    color_message_count="$(
      topic_message_count "${bag_dir}/metadata.yaml" "${COLOR_TOPIC}" || true
    )"
    if [[ ! "${color_message_count}" =~ ^[0-9]+$ ]] ||
       ((color_message_count == 0)); then
      failure_reason="RGB topic ${COLOR_TOPIC} contains no recorded frames"
    elif [[ "${ENABLE_DEPTH}" == "true" ]]; then
      depth_message_count="$(
        topic_message_count "${bag_dir}/metadata.yaml" "${DEPTH_TOPIC}" || true
      )"
      if [[ ! "${depth_message_count}" =~ ^[0-9]+$ ]] ||
         ((depth_message_count == 0)); then
        failure_reason="depth topic ${DEPTH_TOPIC} contains no recorded frames"
      fi
    fi
  fi
  if [[ -n "${failure_reason}" ]]; then
    final_status="incomplete"
  fi
  note="${note//$'\t'/ }"
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${sample_id}" "${note}" "${started_at}" "${DURATION_S}" \
    "${final_status}" "${bag_dir}" >>"${INDEX_FILE}"

  if [[ "${final_status}" != "complete" ]]; then
    print_recording_diagnostics "${bag_dir}"
    fail "sample ${sample_id} is incomplete: ${failure_reason}"
  fi
  if [[ "${ENABLE_DEPTH}" == "true" ]]; then
    log "Saved sample ${sample_id}: ${bag_dir} (RGB=${color_message_count}, depth=${depth_message_count})"
  else
    log "Saved sample ${sample_id}: ${bag_dir} (RGB=${color_message_count})"
  fi
}

log "No drive or serial process was started"
log "Output session: ${SESSION_DIR}"
log "Place the stationary car, type an optional note, and press Enter"
log "Use notes such as straight_1, left_corner_40cm, or cross_offset_right"

SAMPLE_COUNT=0
while ((MAX_SAMPLES == 0 || SAMPLE_COUNT < MAX_SAMPLES)); do
  if [[ -n "${CAMERA_PID}" ]] && ! process_is_running "${CAMERA_PID}"; then
    tail -n 20 "${CAMERA_LOG}" >&2 || true
    fail "camera node exited during the sampling session"
  fi
  printf '[camera-samples] sample note (q to finish)> '
  if ! IFS= read -r SAMPLE_NOTE; then
    printf '\n'
    break
  fi
  case "${SAMPLE_NOTE,,}" in
    q|quit|exit) break ;;
  esac
  ((SAMPLE_COUNT += 1))
  record_sample "${SAMPLE_COUNT}" "${SAMPLE_NOTE}"
done

log "Completed ${SAMPLE_COUNT} sample(s)"
