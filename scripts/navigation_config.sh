#!/usr/bin/env bash

# Load validated startup and recording settings. The Python reader emits only
# tab-separated data; this helper never evaluates configuration as shell code.

load_navigation_config() {
  [[ -f "${CONFIG_HELPER}" ]] || fail "navigation config helper is missing: ${CONFIG_HELPER}"
  [[ -f "${STARTUP_CONFIG}" ]] || fail "startup config is missing: ${STARTUP_CONFIG}"
  [[ -f "${RECORDING_CONFIG}" ]] || fail "recording config is missing: ${RECORDING_CONFIG}"

  local output
  output="$(${CONFIG_HELPER} --startup "${STARTUP_CONFIG}" --recording "${RECORDING_CONFIG}")" ||
    fail "could not load navigation YAML configuration"

  local key
  local value
  local -A settings=()
  local -a topics=()
  while IFS=$'\t' read -r key value; do
    if [[ "${key}" == "topic" ]]; then
      topics+=("${value}")
    else
      settings["${key}"]="${value}"
    fi
  done <<<"${output}"

  SERIAL_DEVICE="${settings[serial_device]}"
  SERIAL_BAUD_RATE="${settings[serial_baud_rate]}"
  USE_SERIAL="${settings[use_serial]}"
  CAMERA_SERIAL="${settings[camera_serial]#_}"
  USE_IMU="${settings[use_imu]}"
  USE_SLAM_IMU="${settings[use_slam_imu]}"
  EQUALIZE="${settings[equalize]}"
  VISUALIZATION="${settings[visualization]}"
  BUILD_IF_NEEDED="${settings[build_if_needed]}"
  CHECK_CAMERA="${settings[check_camera]}"
  FORCE_CAMERA_RESET="${settings[force_camera_reset]}"
  BUILD_JOBS="${settings[build_jobs]}"
  RETAIN_BAGS="${settings[retain_bags]}"
  DEFAULT_AUTOSTART_ROUTE="${settings[default_autostart_route]}"
  ((${#topics[@]} > 0)) || fail "recording config contains no topics"
  ROSBAG_TOPICS=("${topics[@]}")

  [[ "${DEFAULT_AUTOSTART_ROUTE}" = /* ]] ||
    DEFAULT_AUTOSTART_ROUTE="${WORKSPACE_ROOT}/${DEFAULT_AUTOSTART_ROUTE}"
}
