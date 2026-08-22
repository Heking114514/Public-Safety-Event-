#!/usr/bin/env bash

set -Eeuo pipefail

ORIGINAL_ARGV=("$@")
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
ORB_ROOT="${WORKSPACE_ROOT}/src/ORB_SLAM3"
DEPS_ROOT="${WORKSPACE_ROOT}/src/deps"
RUN_MANIFEST_HELPER="${SCRIPT_DIR}/navigation_run_manifest.py"
BUILD_MANIFEST="${NAVIGATION_BUILD_MANIFEST:-${WORKSPACE_ROOT}/build/navigation_runtime_manifest.json}"
RUNS_ROOT="${NAVIGATION_RUNS_ROOT:-${WORKSPACE_ROOT}/navigation_runs}"
LATEST_RUN="${NAVIGATION_LATEST_RUN:-${WORKSPACE_ROOT}/latest_navigation_run}"
LATEST_BAG="${NAVIGATION_LATEST_BAG:-${WORKSPACE_ROOT}/latest_navigation_bag}"
ROS2_COMMAND="${NAVIGATION_ROS2_COMMAND:-ros2}"
PGREP_COMMAND="${NAVIGATION_PGREP_COMMAND:-pgrep}"
CMAKE_COMMAND="${NAVIGATION_CMAKE_COMMAND:-cmake}"
COLCON_COMMAND="${NAVIGATION_COLCON_COMMAND:-colcon}"

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
ROSBAG_OUTPUT=""
ROSBAG_TOPICS=(
  # Raw stereo streams are required to diagnose ORB-SLAM quality, sync and
  # calibration during replay. Keep both image and camera_info topics.
  /camera/camera/infra1/image_rect_raw
  /camera/camera/infra1/camera_info
  /camera/camera/infra2/image_rect_raw
  /camera/camera/infra2/camera_info
  /odometry/visual_continuous
  /odometry/visual_raw
  /orbslam3/map_change
  /wheel/odom
  /odometry/local
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
  /cup_car_serial/actuator_healthy
  /cup_car_serial/actuator_tracking_status
  /cup_car_serial/control_telemetry
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
  --no-build                Use only a build verified against current source
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
[[ -x "${RUN_MANIFEST_HELPER}" ]] ||
  fail "run manifest helper is missing or not executable: ${RUN_MANIFEST_HELPER}"
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
  "${PGREP_COMMAND}" -f "${pattern}" 2>/dev/null || true
}

is_arena_planner_pid() {
  local pid="$1"
  ps -p "${pid}" -o args= 2>/dev/null | grep -Eq 'arena_path_planner|start_arena_planner\.sh'
}

stop_existing_ros_nodes() {
  local -a pids=()
  local -a remaining=()
  local attempt

  mapfile -t pids < <(find_ros_processes)
  # The arena planner is a separate service used by route_frontend.py. Do not
  # terminate it when refreshing the camera/odometry/navigation stack.
  local -a kept_pids=()
  for pid in "${pids[@]}"; do
    if [[ -n "${pid}" ]] && is_arena_planner_pid "${pid}"; then
      kept_pids+=("${pid}")
    fi
  done
  if ((${#kept_pids[@]} > 0)); then
    local -a filtered_pids=()
    for pid in "${pids[@]}"; do
      [[ " ${kept_pids[*]} " == *" ${pid} "* ]] || filtered_pids+=("${pid}")
    done
    pids=("${filtered_pids[@]}")
    log "Keeping ${#kept_pids[@]} arena planner process(es)"
  fi
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

  "${ROS2_COMMAND}" daemon stop >/dev/null 2>&1 || true

  mapfile -t remaining < <(find_ros_processes)
  local -a unexpected_remaining=()
  for pid in "${remaining[@]}"; do
    [[ -n "${pid}" ]] || continue
    is_arena_planner_pid "${pid}" || unexpected_remaining+=("${pid}")
  done
  ((${#unexpected_remaining[@]} == 0)) ||
    fail "could not stop all existing ROS 2 processes: ${unexpected_remaining[*]}"
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

  if "${PGREP_COMMAND}" -f '/realsense2_camera_node([[:space:]]|$)' >/dev/null 2>&1; then
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

  log "Building DBoW2 incrementally"
  "${CMAKE_COMMAND}" -S "${ORB_ROOT}/Thirdparty/DBoW2" -B "${dbow_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  "${CMAKE_COMMAND}" --build "${dbow_build}" --parallel "${BUILD_JOBS}"

  log "Building g2o incrementally"
  "${CMAKE_COMMAND}" -S "${ORB_ROOT}/Thirdparty/g2o" -B "${g2o_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=14 \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  "${CMAKE_COMMAND}" --build "${g2o_build}" --parallel "${BUILD_JOBS}"

  log "Building ORB-SLAM3"
  "${CMAKE_COMMAND}" -S "${ORB_ROOT}" -B "${orb_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DPangolin_DIR="${DEPS_ROOT}/lib/cmake/Pangolin"
  "${CMAKE_COMMAND}" --build "${orb_build}" --parallel "${BUILD_JOBS}" --target ORB_SLAM3
}

build_ros_packages() {
  log "Building ROS 2 nodes"
  local packages=(mission_control_interfaces orbslam3 imu_rpy_filter wheel_odometry fused_odometry visual_navigation)
  if [[ "${USE_SERIAL}" == "true" ]]; then
    packages+=(cup_car_serial)
  fi

  cd "${WORKSPACE_ROOT}"
  CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}" "${COLCON_COMMAND}" build --symlink-install \
    --executor sequential \
    --packages-ignore ORB_SLAM3 pangolin \
    --packages-select "${packages[@]}" \
    --cmake-args \
      -DSophus_DIR="${DEPS_ROOT}/share/sophus/cmake" \
      -DPangolin_DIR="${DEPS_ROOT}/lib/cmake/Pangolin"
}

runtime_artifacts() {
  RUNTIME_ARTIFACTS=(
    "orb_slam3=${ORB_ROOT}/lib/libORB_SLAM3.so"
    "orb_vocabulary=${WORKSPACE_ROOT}/install/orbslam3/share/orbslam3/vocabulary/ORBvoc.txt"
    "dbow2=${ORB_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so"
    "g2o=${ORB_ROOT}/Thirdparty/g2o/lib/libg2o.so"
    "pangolin_core=${DEPS_ROOT}/lib/libpango_core.so"
    "stereo_inertial=${WORKSPACE_ROOT}/install/orbslam3/lib/orbslam3/stereo-inertial"
    "fusion_gate=${WORKSPACE_ROOT}/install/fused_odometry/lib/fused_odometry/fusion_gate_node"
    "map_odom_correction=${WORKSPACE_ROOT}/install/fused_odometry/lib/fused_odometry/map_odom_correction_node"
    "waypoint_navigator=${WORKSPACE_ROOT}/install/visual_navigation/lib/visual_navigation/waypoint_navigator"
    "mission_typesupport=${WORKSPACE_ROOT}/install/mission_control_interfaces/lib/libmission_control_interfaces__rosidl_typesupport_cpp.so"
    "robot_localization_ekf=/opt/ros/${ROS_DISTRO_NAME}/lib/robot_localization/ekf_node"
    "odometry_launch=${WORKSPACE_ROOT}/install/fused_odometry/share/fused_odometry/launch/odometry_bringup.launch.py"
    "fusion_launch=${WORKSPACE_ROOT}/install/fused_odometry/share/fused_odometry/launch/fused_odometry.launch.py"
    "fusion_config=${WORKSPACE_ROOT}/install/fused_odometry/share/fused_odometry/config/fused_odometry.yaml"
    "orb_launch=${WORKSPACE_ROOT}/install/orbslam3/share/orbslam3/launch/realsense_d455_stereo_inertial.launch.py"
    "orb_config=${WORKSPACE_ROOT}/install/orbslam3/share/orbslam3/config/stereo-inertial/RealSense_D455.yaml"
    "navigation_base_launch=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/launch/visual_navigation_bringup.launch.py"
    "waypoint_launch=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/launch/waypoint_navigation.launch.py"
    "navigation_config=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/config/waypoint_navigation.yaml"
  )
  if [[ "${USE_IMU}" == "true" ]]; then
    RUNTIME_ARTIFACTS+=(
      "imu_rpy_filter=${WORKSPACE_ROOT}/install/imu_rpy_filter/lib/imu_rpy_filter/imu_rpy_filter_node"
      "imu_config=${WORKSPACE_ROOT}/install/imu_rpy_filter/share/imu_rpy_filter/config/imu_rpy_filter.yaml"
    )
  fi
  if [[ "${USE_SERIAL}" == "true" ]]; then
    RUNTIME_ARTIFACTS+=(
      "wheel_odometry=${WORKSPACE_ROOT}/install/wheel_odometry/lib/wheel_odometry/wheel_odometry_node"
      "cmd_vel_serial=${WORKSPACE_ROOT}/install/cup_car_serial/lib/cup_car_serial/cmd_vel_serial_node"
      "wheel_config=${WORKSPACE_ROOT}/install/wheel_odometry/share/wheel_odometry/config/wheel_odometry.yaml"
      "navigation_serial_launch=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/launch/visual_navigation_serial_bringup.launch.py"
    )
  fi
}

runtime_artifacts
SOURCE_FINGERPRINT_BEFORE="$(${RUN_MANIFEST_HELPER} fingerprint --workspace "${WORKSPACE_ROOT}")" ||
  fail "could not compute the runtime source fingerprint"

declare -a ARTIFACT_ARGUMENTS=()
for artifact in "${RUNTIME_ARTIFACTS[@]}"; do
  ARTIFACT_ARGUMENTS+=(--artifact "${artifact}")
done

if [[ "${BUILD_IF_NEEDED}" == "true" ]]; then
  build_orb_slam3
  build_ros_packages
  SOURCE_FINGERPRINT_AFTER="$(${RUN_MANIFEST_HELPER} fingerprint --workspace "${WORKSPACE_ROOT}")" ||
    fail "could not recompute the runtime source fingerprint after building"
  [[ "${SOURCE_FINGERPRINT_AFTER}" == "${SOURCE_FINGERPRINT_BEFORE}" ]] ||
    fail "runtime source changed during the build; rerun after edits stop"
  BUILD_ID="$(${RUN_MANIFEST_HELPER} record-build \
    --workspace "${WORKSPACE_ROOT}" \
    --output "${BUILD_MANIFEST}" \
    --expected-fingerprint "${SOURCE_FINGERPRINT_AFTER}" \
    --setting "ros_distro=${ROS_DISTRO_NAME}" \
    --setting "use_imu=${USE_IMU}" \
    --setting "use_serial=${USE_SERIAL}" \
    "${ARTIFACT_ARGUMENTS[@]}")" ||
    fail "could not record the successful runtime build manifest"
  SOURCE_FINGERPRINT="${SOURCE_FINGERPRINT_AFTER}"
else
  BUILD_ID="$(${RUN_MANIFEST_HELPER} verify-build \
    --workspace "${WORKSPACE_ROOT}" \
    --manifest "${BUILD_MANIFEST}" \
    "${ARTIFACT_ARGUMENTS[@]}")" ||
    fail "--no-build verification failed; rerun without --no-build"
  SOURCE_FINGERPRINT="${SOURCE_FINGERPRINT_BEFORE}"
fi

stop_existing_ros_nodes

if [[ "${CHECK_CAMERA}" == "true" ]]; then
  detect_camera_serial
elif [[ -z "${CAMERA_SERIAL}" ]]; then
  CAMERA_SERIAL="038122250473"
  log "Camera check skipped; using configured default serial ${CAMERA_SERIAL}"
fi

prepare_camera_imu

[[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]] || fail "workspace setup was not generated"
# shellcheck disable=SC1091
set +u
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

export LD_LIBRARY_PATH="${ORB_ROOT}/lib:${DEPS_ROOT}/lib:${LD_LIBRARY_PATH:-}"

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

declare -a RUN_INPUT_FILES=(
  "startup_script=${SCRIPT_DIR}/start_visual_navigation.sh"
  "manifest_helper=${RUN_MANIFEST_HELPER}"
  "odometry_launch=${WORKSPACE_ROOT}/install/fused_odometry/share/fused_odometry/launch/odometry_bringup.launch.py"
  "fusion_launch=${WORKSPACE_ROOT}/install/fused_odometry/share/fused_odometry/launch/fused_odometry.launch.py"
  "fusion_config=${WORKSPACE_ROOT}/install/fused_odometry/share/fused_odometry/config/fused_odometry.yaml"
  "orb_launch=${WORKSPACE_ROOT}/install/orbslam3/share/orbslam3/launch/realsense_d455_stereo_inertial.launch.py"
  "orb_config=${WORKSPACE_ROOT}/install/orbslam3/share/orbslam3/config/stereo-inertial/RealSense_D455.yaml"
  "navigation_launch=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/launch/${LAUNCH_FILE}"
  "navigation_base_launch=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/launch/visual_navigation_bringup.launch.py"
  "waypoint_launch=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/launch/waypoint_navigation.launch.py"
  "navigation_config=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/config/waypoint_navigation.yaml"
)
if [[ "${USE_IMU}" == "true" ]]; then
  RUN_INPUT_FILES+=(
    "imu_config=${WORKSPACE_ROOT}/install/imu_rpy_filter/share/imu_rpy_filter/config/imu_rpy_filter.yaml"
  )
fi
if [[ "${USE_SERIAL}" == "true" ]]; then
  RUN_INPUT_FILES+=(
    "wheel_config=${WORKSPACE_ROOT}/install/wheel_odometry/share/wheel_odometry/config/wheel_odometry.yaml"
    "navigation_serial_launch=${WORKSPACE_ROOT}/install/visual_navigation/share/visual_navigation/launch/visual_navigation_serial_bringup.launch.py"
  )
fi

declare -a CREATE_RUN_ARGUMENTS=(
  create-run
  --workspace "${WORKSPACE_ROOT}"
  --runs-root "${RUNS_ROOT}"
  --latest-bag "${LATEST_BAG}"
  --latest-run "${LATEST_RUN}"
  --source-fingerprint "${SOURCE_FINGERPRINT}"
  --build-manifest "${BUILD_MANIFEST}"
  --build-id "${BUILD_ID}"
  --setting "ros_distro=${ROS_DISTRO_NAME}"
  --setting "camera_serial=${CAMERA_SERIAL}"
  --setting "camera_initial_reset=${CAMERA_INITIAL_RESET}"
  --setting "check_camera=${CHECK_CAMERA}"
  --setting "force_camera_reset=${FORCE_CAMERA_RESET}"
  --setting "serial_device=${SERIAL_DEVICE}"
  --setting "serial_baud_rate=${SERIAL_BAUD_RATE}"
  --setting "use_serial=${USE_SERIAL}"
  --setting "use_imu=${USE_IMU}"
  --setting "use_slam_imu=${USE_SLAM_IMU}"
  --setting "equalize=${EQUALIZE}"
  --setting "visualization=${VISUALIZATION}"
  --setting "autostart=${AUTOSTART}"
  --setting "build_enabled=${BUILD_IF_NEEDED}"
  --setting "build_jobs=${BUILD_JOBS}"
  --setting "launch_file=${LAUNCH_FILE}"
  --setting "route_source=dynamic:/waypoint_navigation/route_input"
)
for argument in "${ORIGINAL_ARGV[@]}"; do
  CREATE_RUN_ARGUMENTS+=("--argv=${argument}")
done
for topic in "${ROSBAG_TOPICS[@]}"; do
  CREATE_RUN_ARGUMENTS+=(--topic "${topic}")
done
for input_file in "${RUN_INPUT_FILES[@]}"; do
  CREATE_RUN_ARGUMENTS+=(--input-file "${input_file}")
done

RUN_DIR="$(${RUN_MANIFEST_HELPER} "${CREATE_RUN_ARGUMENTS[@]}")" ||
  fail "could not create the navigation run manifest"
[[ -n "${RUN_DIR}" && -d "${RUN_DIR}" ]] || fail "run directory was not created"
PARAMETERS_DIR="${RUN_DIR}/parameters"
ROSBAG_OUTPUT="${RUN_DIR}/bag"
[[ ! -e "${ROSBAG_OUTPUT}" && ! -L "${ROSBAG_OUTPUT}" ]] ||
  fail "rosbag output already exists: ${ROSBAG_OUTPUT}"

declare -a STACK_PIDS=()
STACK_STOPPED="false"
RUN_FINALIZED="false"
PARAMETER_SNAPSHOTS_COMPLETE="false"

stop_stack() {
  [[ "${STACK_STOPPED}" == "false" ]] || return 0
  STACK_STOPPED="true"
  if ((${#STACK_PIDS[@]} > 0)); then
    local -a remaining=()
    local attempt
    local pid

    timeout 1.0 "${ROS2_COMMAND}" service call \
      /waypoint_navigator/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
    timeout 1.5 "${ROS2_COMMAND}" topic pub -r 20 -t 4 -w 0 \
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

stack_process_exited() {
  local pid
  for pid in "${STACK_PIDS[@]}"; do
    kill -0 "${pid}" 2>/dev/null || return 0
  done
  return 1
}

record_parameter_status() {
  "${RUN_MANIFEST_HELPER}" record-parameter --run-dir "${RUN_DIR}" "$@" ||
    fail "could not update the run manifest with a parameter snapshot"
}

snapshot_runtime_parameters() {
  local -a active_nodes=(
    /fused_odometry_gate
    /fused_ekf
    /map_odom_correction
    /waypoint_navigator
    /orbslam3_stereo_inertial
    /camera/camera
  )
  local node
  if [[ "${USE_IMU}" == "true" ]]; then
    active_nodes+=(/imu_rpy_filter)
  else
    record_parameter_status --node /imu_rpy_filter --status skipped \
      --error "disabled by --no-imu"
  fi
  if [[ "${USE_SERIAL}" == "true" ]]; then
    active_nodes+=(/wheel_odometry_node /cmd_vel_serial_node)
  else
    record_parameter_status --node /wheel_odometry_node --status skipped \
      --error "disabled because serial mode is off"
    record_parameter_status --node /cmd_vel_serial_node --status skipped \
      --error "disabled by --no-serial"
  fi

  local deadline=$((SECONDS + 30))
  local node_list=""
  local all_visible
  while ((SECONDS < deadline)); do
    node_list="$(timeout 2.0 "${ROS2_COMMAND}" node list 2>/dev/null || true)"
    all_visible="true"
    for node in "${active_nodes[@]}"; do
      if ! grep -Fxq -- "${node}" <<<"${node_list}"; then
        all_visible="false"
        break
      fi
    done
    [[ "${all_visible}" == "true" ]] && break
    stack_process_exited && break
    sleep 0.25
  done

  local snapshot_failed="false"
  local safe_name
  local snapshot
  local temporary
  local dump_status
  for node in "${active_nodes[@]}"; do
    if ! grep -Fxq -- "${node}" <<<"${node_list}"; then
      record_parameter_status --node "${node}" --status failed \
        --error "node did not become visible within the startup timeout"
      snapshot_failed="true"
      continue
    fi

    safe_name="${node#/}"
    safe_name="${safe_name//\//_}"
    snapshot="${PARAMETERS_DIR}/${safe_name}.yaml"
    temporary="${snapshot}.tmp"
    set +e
    timeout 6.0 "${ROS2_COMMAND}" param dump --no-daemon --spin-time 0.5 "${node}" \
      >"${temporary}" 2>"${temporary}.stderr"
    dump_status=$?
    set -e
    if ((dump_status == 0)) && [[ -s "${temporary}" ]]; then
      mv -f -- "${temporary}" "${snapshot}"
      rm -f -- "${temporary}.stderr"
      record_parameter_status --node "${node}" --status success --snapshot "${snapshot}"
    else
      local dump_error=""
      if [[ -s "${temporary}.stderr" ]]; then
        dump_error="$(<"${temporary}.stderr")"
        dump_error="${dump_error//$'\n'/ }"
        dump_error="${dump_error:0:512}"
      fi
      rm -f -- "${temporary}" "${temporary}.stderr"
      record_parameter_status --node "${node}" --status failed \
        --error "ros2 param dump exited with status ${dump_status}: ${dump_error}"
      snapshot_failed="true"
    fi
  done

  if [[ "${snapshot_failed}" == "false" ]]; then
    PARAMETER_SNAPSHOTS_COMPLETE="true"
  fi
}

cleanup() {
  local original_status=$?
  trap - EXIT INT TERM
  set +e
  local final_status="${original_status}"

  stop_stack
  if [[ "${RUN_FINALIZED}" == "false" ]]; then
    RUN_FINALIZED="true"
    "${RUN_MANIFEST_HELPER}" finalize-run \
      --run-dir "${RUN_DIR}" \
      --latest-bag "${LATEST_BAG}" \
      --exit-code "${original_status}" \
      --parameter-snapshots-complete "${PARAMETER_SNAPSHOTS_COMPLETE}"
    local finalize_status=$?
    if ((finalize_status != 0)); then
      printf '[visual-navigation] ERROR: could not finalize run manifest %s\n' \
        "${RUN_DIR}/run_manifest.json" >&2
      ((final_status != 0)) || final_status=1
    fi
  fi
  exit "${final_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"${ROS2_COMMAND}" launch fused_odometry odometry_bringup.launch.py \
  "${ODOMETRY_LAUNCH_ARGS[@]}" &
STACK_PIDS+=("$!")

"${ROS2_COMMAND}" launch visual_navigation "${LAUNCH_FILE}" \
  "${NAVIGATION_LAUNCH_ARGS[@]}" &
STACK_PIDS+=("$!")

log "Recording latest navigation data: ${ROSBAG_OUTPUT}"
"${ROS2_COMMAND}" bag record --output "${ROSBAG_OUTPUT}" "${ROSBAG_TOPICS[@]}" &
STACK_PIDS+=("$!")
"${RUN_MANIFEST_HELPER}" mark-running --run-dir "${RUN_DIR}" ||
  fail "could not mark the navigation run as started"

snapshot_runtime_parameters

set +e
wait -n "${STACK_PIDS[@]}"
STACK_STATUS=$?
set -e
log "A stack process exited with status ${STACK_STATUS}; stopping the remaining stack"
exit "${STACK_STATUS}"
