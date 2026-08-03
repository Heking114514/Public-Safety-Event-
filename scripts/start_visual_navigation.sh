#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
ORB_ROOT="${WORKSPACE_ROOT}/src/ORB_SLAM3"
DEPS_ROOT="${WORKSPACE_ROOT}/src/deps"

ROUTE_FILE="${WORKSPACE_ROOT}/src/visual_navigation/routes/example_route.csv"
SERIAL_DEVICE="auto"
SERIAL_BAUD_RATE="115200"
USE_SERIAL="true"
CAMERA_SERIAL=""
USE_IMU="true"
VISUALIZATION="false"
AUTOSTART="false"
BUILD_IF_NEEDED="true"
CHECK_CAMERA="true"
FORCE_CAMERA_RESET="true"
CAMERA_INITIAL_RESET="false"
BUILD_JOBS="2"

log() {
  printf '[visual-navigation] %s\n' "$*"
}

fail() {
  printf '[visual-navigation] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: scripts/start_visual_navigation.sh [options]

Options:
  --route PATH              CSV route file
  --camera-serial SERIAL    D455 serial number; auto-detected by default
  --serial-device DEVICE    Controller serial device (default: auto)
  --serial-baud RATE        Controller baud rate (default: 115200)
  --no-serial               Run upper-computer algorithms without the controller
  --no-imu                  Use stereo mode without IMU initialization
  --visualization           Enable the Pangolin window
  --autostart               Start waypoint motion immediately (unsafe on a bench)
  --no-build                Fail instead of building missing binaries
  --skip-camera-check       Launch without checking for a connected D455
  --reset-camera            Force a D455 firmware reset before opening streams (default with IMU)
  --no-camera-reset         Skip the default reset if the D455 is already known to be healthy
  --jobs COUNT              Parallel build jobs
  -h, --help                Show this help

Navigation does not move until the start service is called unless --autostart is used:
  ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
EOF
}

while (($# > 0)); do
  case "$1" in
    --route)
      (($# >= 2)) || fail "--route requires a path"
      ROUTE_FILE="$2"
      shift 2
      ;;
    --camera-serial)
      (($# >= 2)) || fail "--camera-serial requires a serial number"
      CAMERA_SERIAL="${2#_}"
      shift 2
      ;;
    --serial-device)
      (($# >= 2)) || fail "--serial-device requires a device"
      SERIAL_DEVICE="$2"
      shift 2
      ;;
    --serial-baud)
      (($# >= 2)) || fail "--serial-baud requires a rate"
      SERIAL_BAUD_RATE="$2"
      shift 2
      ;;
    --no-serial)
      USE_SERIAL="false"
      shift
      ;;
    --no-imu)
      USE_IMU="false"
      shift
      ;;
    --visualization)
      VISUALIZATION="true"
      shift
      ;;
    --autostart)
      AUTOSTART="true"
      shift
      ;;
    --no-build)
      BUILD_IF_NEEDED="false"
      shift
      ;;
    --skip-camera-check)
      CHECK_CAMERA="false"
      shift
      ;;
    --reset-camera)
      FORCE_CAMERA_RESET="true"
      shift
      ;;
    --no-camera-reset)
      FORCE_CAMERA_RESET="false"
      shift
      ;;
    --jobs)
      (($# >= 2)) || fail "--jobs requires a count"
      BUILD_JOBS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown option: $1 (use --help)"
      ;;
  esac
done

[[ -f "${ROS_SETUP}" ]] || fail "ROS 2 setup not found: ${ROS_SETUP}"
[[ -f "${ROUTE_FILE}" ]] || fail "route file not found: ${ROUTE_FILE}"
[[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || fail "--jobs must be a positive integer"
[[ "${SERIAL_BAUD_RATE}" =~ ^[0-9]+$ ]] || fail "--serial-baud must be an integer"
[[ -f "${ORB_ROOT}/CMakeLists.txt" ]] || fail "ORB_SLAM3 submodule is missing; run: git submodule update --init --recursive"

# shellcheck disable=SC1090
set +u
source "${ROS_SETUP}"
set -u

find_ros_processes() {
  local pattern
  pattern="/opt/ros/${ROS_DISTRO_NAME}/bin/ros2([[:space:]]|$)"
  pattern+="|/opt/ros/${ROS_DISTRO_NAME}/lib/[^[:space:]]+/[^[:space:]]+"
  pattern+="|${WORKSPACE_ROOT}/install/[^[:space:]]+/lib/[^[:space:]]+/[^[:space:]]+"
  pgrep -f "${pattern}" 2>/dev/null || true
}

stop_existing_ros_nodes() {
  local -a pids=()
  local -a remaining=()
  local attempt

  mapfile -t pids < <(find_ros_processes)
  if ((${#pids[@]} == 0)); then
    log "No existing ROS 2 nodes found"
  else
    log "Stopping ${#pids[@]} existing ROS 2 processes"
    kill -INT "${pids[@]}" 2>/dev/null || true

    for attempt in {1..40}; do
      remaining=()
      for pid in "${pids[@]}"; do
        kill -0 "${pid}" 2>/dev/null && remaining+=("${pid}")
      done
      ((${#remaining[@]} == 0)) && break
      sleep 0.2
    done

    if ((${#remaining[@]} > 0)); then
      log "Forcing ${#remaining[@]} ROS 2 processes to exit"
      kill -TERM "${remaining[@]}" 2>/dev/null || true
      for attempt in {1..20}; do
        pids=()
        for pid in "${remaining[@]}"; do
          kill -0 "${pid}" 2>/dev/null && pids+=("${pid}")
        done
        ((${#pids[@]} == 0)) && break
        sleep 0.1
      done
      ((${#pids[@]} == 0)) || kill -KILL "${pids[@]}" 2>/dev/null || true
    fi
  fi

  ros2 daemon stop >/dev/null 2>&1 || true

  mapfile -t remaining < <(find_ros_processes)
  ((${#remaining[@]} == 0)) ||
    fail "could not stop all existing ROS 2 processes: ${remaining[*]}"
}

detect_camera_serial() {
  command -v rs-enumerate-devices >/dev/null 2>&1 ||
    fail "rs-enumerate-devices is unavailable; install ros-${ROS_DISTRO_NAME}-realsense2-camera"

  local devices
  local device_status
  set +e
  devices="$(rs-enumerate-devices 2>&1)"
  device_status=$?
  set -e
  if grep -q "No device detected" <<<"${devices}"; then
    fail "D455 not detected; reconnect it to a USB 3.x port"
  fi
  ((device_status == 0)) || fail "RealSense device query failed: ${devices}"

  if [[ -z "${CAMERA_SERIAL}" ]]; then
    CAMERA_SERIAL="$(awk -F: '/^[[:space:]]*Serial Number[[:space:]]*:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${devices}")"
  fi
  [[ -n "${CAMERA_SERIAL}" ]] || fail "could not read the D455 serial number"
  log "D455 detected: ${CAMERA_SERIAL}"
}

prepare_camera_imu() {
  [[ "${USE_IMU}" == "true" ]] || return 0

  if pgrep -f '/realsense2_camera_node([[:space:]]|$)' >/dev/null 2>&1; then
    fail "a RealSense camera node is already running; stop it before starting another one"
  fi

  local accel_found="false"
  local gyro_found="false"
  local stale_buffer="false"
  local device
  local device_path
  local sensor_name
  local buffer_file
  local buffer_state

  for device in /sys/bus/iio/devices/iio:device*; do
    [[ -r "${device}/name" ]] || continue
    device_path="$(readlink -f "${device}")"
    [[ "${device_path}" == *"8086:0B5C"* ]] || continue

    sensor_name="$(<"${device}/name")"
    case "${sensor_name}" in
      accel_3d) accel_found="true" ;;
      gyro_3d) gyro_found="true" ;;
      *) continue ;;
    esac

    buffer_file="${device}/buffer/enable"
    [[ -r "${buffer_file}" ]] || continue
    buffer_state="$(<"${buffer_file}")"
    if [[ "${buffer_state}" == "1" ]]; then
      printf '0' >"${buffer_file}" ||
        fail "could not clear stale IMU buffer: ${buffer_file}"
      stale_buffer="true"
    fi
  done

  if [[ "${FORCE_CAMERA_RESET}" == "true" ]]; then
    CAMERA_INITIAL_RESET="true"
    log "D455 one-time firmware reset enabled for reliable IMU startup"
  elif [[ "${accel_found}" != "true" ||
        "${gyro_found}" != "true" ||
        "${stale_buffer}" == "true" ]]; then
    CAMERA_INITIAL_RESET="true"
    log "D455 IMU needs recovery; enabling one-time firmware reset"
  else
    log "D455 IMU preflight passed"
  fi
}

build_orb_slam3() {
  local dbow_build="${ORB_ROOT}/Thirdparty/DBoW2/build"
  local g2o_build="${ORB_ROOT}/Thirdparty/g2o/build"
  local orb_build="${ORB_ROOT}/build"

  if [[ ! -f "${ORB_ROOT}/Vocabulary/ORBvoc.txt" ]]; then
    log "Extracting ORB-SLAM3 vocabulary"
    tar -xf "${ORB_ROOT}/Vocabulary/ORBvoc.txt.tar.gz" -C "${ORB_ROOT}/Vocabulary"
  fi

  if [[ ! -f "${ORB_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so" ]]; then
    log "Building DBoW2"
    cmake -S "${ORB_ROOT}/Thirdparty/DBoW2" -B "${dbow_build}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    cmake --build "${dbow_build}" --parallel "${BUILD_JOBS}"
  fi

  if [[ ! -f "${ORB_ROOT}/Thirdparty/g2o/lib/libg2o.so" ]]; then
    log "Building g2o"
    cmake -S "${ORB_ROOT}/Thirdparty/g2o" -B "${g2o_build}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=14 \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    cmake --build "${g2o_build}" --parallel "${BUILD_JOBS}"
  fi

  log "Building ORB-SLAM3"
  cmake -S "${ORB_ROOT}" -B "${orb_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DPangolin_DIR="${DEPS_ROOT}/lib/cmake/Pangolin"
  cmake --build "${orb_build}" --parallel "${BUILD_JOBS}" --target ORB_SLAM3
}

build_ros_packages() {
  log "Building ROS 2 nodes"
  local packages=(orbslam3 imu_rpy_filter visual_navigation)
  if [[ "${USE_SERIAL}" == "true" ]]; then
    packages+=(cup_car_serial)
  fi

  cd "${WORKSPACE_ROOT}"
  colcon build --symlink-install \
    --packages-ignore ORB_SLAM3 pangolin \
    --packages-select "${packages[@]}" \
    --cmake-args \
      -DSophus_DIR="${DEPS_ROOT}/share/sophus/cmake" \
      -DPangolin_DIR="${DEPS_ROOT}/lib/cmake/Pangolin"
}

runtime_ready() {
    [[ -f "${ORB_ROOT}/lib/libORB_SLAM3.so" ]] &&
    [[ -x "${WORKSPACE_ROOT}/install/orbslam3/lib/orbslam3/stereo-inertial" ]] &&
    [[ -x "${WORKSPACE_ROOT}/install/imu_rpy_filter/lib/imu_rpy_filter/imu_rpy_filter_node" ]] &&
    [[ -x "${WORKSPACE_ROOT}/install/visual_navigation/lib/visual_navigation/waypoint_navigator" ]] &&
    { [[ "${USE_SERIAL}" == "false" ]] ||
      [[ -x "${WORKSPACE_ROOT}/install/cup_car_serial/lib/cup_car_serial/cmd_vel_serial_node" ]]; }
}

stop_existing_ros_nodes

if [[ "${CHECK_CAMERA}" == "true" ]]; then
  detect_camera_serial
elif [[ -z "${CAMERA_SERIAL}" ]]; then
  CAMERA_SERIAL="038122250473"
  log "Camera check skipped; using configured default serial ${CAMERA_SERIAL}"
fi

if ! runtime_ready; then
  [[ "${BUILD_IF_NEEDED}" == "true" ]] ||
    fail "required binaries are missing; rerun without --no-build"
  [[ -f "${ORB_ROOT}/lib/libORB_SLAM3.so" ]] || build_orb_slam3
  build_ros_packages
fi

prepare_camera_imu

[[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]] || fail "workspace setup was not generated"
# shellcheck disable=SC1091
set +u
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

export LD_LIBRARY_PATH="${ORB_ROOT}/lib:${DEPS_ROOT}/lib:${LD_LIBRARY_PATH:-}"

log "Starting visual navigation"
log "Route: ${ROUTE_FILE}"
log "IMU: ${USE_IMU}; visualization: ${VISUALIZATION}; autostart: ${AUTOSTART}"
if [[ "${USE_SERIAL}" == "true" ]]; then
  log "Controller serial: ${SERIAL_DEVICE} at ${SERIAL_BAUD_RATE} baud"
  LAUNCH_FILE="visual_navigation_serial_bringup.launch.py"
else
  log "Controller serial: disabled (upper-computer only)"
  LAUNCH_FILE="visual_navigation_bringup.launch.py"
fi

LAUNCH_ARGS=(
  "route_file:=${ROUTE_FILE}"
  "route_frame:=map"
  "body_frame_id:=camera_link"
  "cmd_vel_topic:=/cmd_vel_nav"
  "serial_no:=_${CAMERA_SERIAL}"
  "initial_reset:=${CAMERA_INITIAL_RESET}"
  "visualization:=${VISUALIZATION}"
  "use_imu:=${USE_IMU}"
  "autostart:=${AUTOSTART}"
)

if [[ "${USE_SERIAL}" == "true" ]]; then
  LAUNCH_ARGS+=(
    "serial_device:=${SERIAL_DEVICE}"
    "serial_baud_rate:=${SERIAL_BAUD_RATE}"
  )
fi

exec ros2 launch visual_navigation "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}"
