#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
ORB_ROOT="${WORKSPACE_ROOT}/src/ORB_SLAM3"
DEPS_ROOT="${WORKSPACE_ROOT}/src/deps"

SERIAL_DEVICE="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
SERIAL_BAUD_RATE="115200"
USE_SERIAL="true"
CAMERA_SERIAL=""
USE_IMU="true"
USE_SLAM_IMU="false"
EQUALIZE="true"
VISUALIZATION="false"
AUTOSTART="false"
BUILD_IF_NEEDED="true"
CHECK_CAMERA="true"
FORCE_CAMERA_RESET="false"
CAMERA_INITIAL_RESET="false"
BUILD_JOBS="2"
ROSBAG_OUTPUT="${WORKSPACE_ROOT}/latest_navigation_bag"
ROSBAG_TOPICS=(
  /odom
  /odom/orb_raw
  /wheel/odom
  /odometry/fused
  /fusion/input/visual_odom
  /fusion/input/wheel_odom
  /fusion/input/imu
  /camera/camera/imu
  /imu/filtered
  /imu/rpy
  /tracking_state
  /odometry/fusion_status
  /cmd_vel_nav
  /waypoint_path
  /waypoint_navigation/route_input
  /waypoint_navigation/status
  /waypoint_navigation/current_waypoint
  /waypoint_navigation/motion_hold_state
  /waypoint_navigation/start
  /cup_car_serial/encoder_ticks
  /cup_car_serial/connected
  /cup_car_serial/rx
  /diagnostics
  /tf
  /tf_static
)

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
  --camera-serial SERIAL    D455 serial number; auto-detected by default
  --serial-device DEVICE    Controller serial device (default: CH340 stable path)
  --serial-baud RATE        Controller baud rate (default: 115200)
  --no-serial               Run upper-computer algorithms without the controller
  --no-imu                  Disable the D455 IMU and IMU filter
  --slam-imu                Fuse raw D455 IMU measurements inside ORB-SLAM3
  --no-equalize             Disable CLAHE image enhancement before ORB-SLAM3
  --visualization           Enable the Pangolin window
  --autostart               Start waypoint motion immediately (disabled by default)
  --no-build                Fail instead of building missing binaries
  --skip-camera-check       Launch without checking for a connected D455
  --reset-camera            Force a D455 firmware reset before opening streams
  --no-camera-reset         Skip camera reset (default)
  --jobs COUNT              Parallel build jobs
  -h, --help                Show this help

No CSV route is loaded at startup. Run route_editor.py, click the route, then
press its publish button to load and start the route immediately.
EOF
}

while (($# > 0)); do
  case "$1" in
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
      USE_SLAM_IMU="false"
      shift
      ;;
    --slam-imu)
      USE_SLAM_IMU="true"
      shift
      ;;
    --no-equalize)
      EQUALIZE="false"
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
[[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || fail "--jobs must be a positive integer"
[[ "${SERIAL_BAUD_RATE}" =~ ^[0-9]+$ ]] || fail "--serial-baud must be an integer"
[[ -f "${ORB_ROOT}/CMakeLists.txt" ]] || fail "ORB_SLAM3 submodule is missing; run: git submodule update --init --recursive"
[[ "${USE_SLAM_IMU}" == "false" || "${USE_IMU}" == "true" ]] ||
  fail "--slam-imu requires D455 IMU; remove --no-imu"

# shellcheck disable=SC1090
set +u
source "${ROS_SETUP}"
set -u

[[ -x "/opt/ros/${ROS_DISTRO_NAME}/lib/robot_localization/ekf_node" ]] ||
  fail "robot_localization is missing; install ros-${ROS_DISTRO_NAME}-robot-localization"

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
  local packages=(mission_control_interfaces orbslam3 imu_rpy_filter wheel_odometry fused_odometry visual_navigation)
  if [[ "${USE_SERIAL}" == "true" ]]; then
    packages+=(cup_car_serial)
  fi

  cd "${WORKSPACE_ROOT}"
  CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}" colcon build --symlink-install \
    --executor sequential \
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
    [[ -x "${WORKSPACE_ROOT}/install/wheel_odometry/lib/wheel_odometry/wheel_odometry_node" ]] &&
    [[ -x "${WORKSPACE_ROOT}/install/fused_odometry/lib/fused_odometry/fusion_gate_node" ]] &&
    [[ -f "${WORKSPACE_ROOT}/install/mission_control_interfaces/share/mission_control_interfaces/srv/SetMotionHold.srv" ]] &&
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

[[ "${ROSBAG_OUTPUT}" == "${WORKSPACE_ROOT}/latest_navigation_bag" ]] ||
  fail "refusing to replace unexpected rosbag path: ${ROSBAG_OUTPUT}"
if [[ -e "${ROSBAG_OUTPUT}" || -L "${ROSBAG_OUTPUT}" ]]; then
  log "Removing previous rosbag: ${ROSBAG_OUTPUT}"
  rm -rf -- "${ROSBAG_OUTPUT}"
fi

log "Starting decoupled odometry and navigation stacks"
log "Route: waiting for /waypoint_navigation/route_input from route_editor.py"
log "IMU filter: ${USE_IMU}; SLAM IMU fusion: ${USE_SLAM_IMU}; CLAHE: ${EQUALIZE}; visualization: ${VISUALIZATION}; autostart: ${AUTOSTART}"
if [[ "${USE_SERIAL}" == "true" ]]; then
  log "Controller serial: ${SERIAL_DEVICE} at ${SERIAL_BAUD_RATE} baud"
  LAUNCH_FILE="visual_navigation_serial_bringup.launch.py"
else
  log "Controller serial: disabled (upper-computer only)"
  LAUNCH_FILE="visual_navigation_bringup.launch.py"
fi

ODOMETRY_LAUNCH_ARGS=(
  "serial_no:=_${CAMERA_SERIAL}"
  "initial_reset:=${CAMERA_INITIAL_RESET}"
  "visualization:=${VISUALIZATION}"
  "use_imu:=${USE_IMU}"
  "use_slam_imu:=${USE_SLAM_IMU}"
  "equalize:=${EQUALIZE}"
  "use_wheel:=${USE_SERIAL}"
)

NAVIGATION_LAUNCH_ARGS=(
  "route_frame:=map"
  "odom_topic:=/odometry/fused"
  "fusion_status_topic:=/odometry/fusion_status"
  "cmd_vel_topic:=/cmd_vel_nav"
  "autostart:=${AUTOSTART}"
)

if [[ "${USE_SERIAL}" == "true" ]]; then
  NAVIGATION_LAUNCH_ARGS+=(
    "serial_device:=${SERIAL_DEVICE}"
    "serial_baud_rate:=${SERIAL_BAUD_RATE}"
  )
fi

declare -a STACK_PIDS=()
STACK_STOPPED="false"

stop_stack() {
  [[ "${STACK_STOPPED}" == "false" ]] || return 0
  STACK_STOPPED="true"
  if ((${#STACK_PIDS[@]} > 0)); then
    local -a remaining=()
    local attempt
    local pid

    timeout 1.0 ros2 service call \
      /waypoint_navigator/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
    timeout 1.5 ros2 topic pub -r 20 -t 4 -w 0 \
      /cmd_vel_nav geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true

    kill -INT "${STACK_PIDS[@]}" 2>/dev/null || true
    # Give rosbag enough time to flush metadata and close its database.
    for attempt in {1..80}; do
      remaining=()
      for pid in "${STACK_PIDS[@]}"; do
        kill -0 "${pid}" 2>/dev/null && remaining+=("${pid}")
      done
      ((${#remaining[@]} == 0)) && break
      sleep 0.1
    done

    if ((${#remaining[@]} > 0)); then
      kill -TERM "${remaining[@]}" 2>/dev/null || true
      for attempt in {1..10}; do
        STACK_PIDS=()
        for pid in "${remaining[@]}"; do
          kill -0 "${pid}" 2>/dev/null && STACK_PIDS+=("${pid}")
        done
        ((${#STACK_PIDS[@]} == 0)) && break
        sleep 0.1
      done
      ((${#STACK_PIDS[@]} == 0)) || kill -KILL "${STACK_PIDS[@]}" 2>/dev/null || true
    fi

    for pid in "${STACK_PIDS[@]}"; do
      wait "${pid}" 2>/dev/null || true
    done
  fi
}

trap stop_stack EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

ros2 launch fused_odometry odometry_bringup.launch.py \
  "${ODOMETRY_LAUNCH_ARGS[@]}" &
STACK_PIDS+=("$!")

ros2 launch visual_navigation "${LAUNCH_FILE}" \
  "${NAVIGATION_LAUNCH_ARGS[@]}" &
STACK_PIDS+=("$!")

log "Recording latest navigation data: ${ROSBAG_OUTPUT}"
ros2 bag record --output "${ROSBAG_OUTPUT}" "${ROSBAG_TOPICS[@]}" &
STACK_PIDS+=("$!")

set +e
wait -n "${STACK_PIDS[@]}"
STACK_STATUS=$?
set -e
log "A stack process exited with status ${STACK_STATUS}; stopping the remaining stack"
exit "${STACK_STATUS}"
