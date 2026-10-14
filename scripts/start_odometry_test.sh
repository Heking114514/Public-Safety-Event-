#!/usr/bin/env bash
# Start the localization/serial/recording stack and a measured straight-line test.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_COMMAND="${ODOMETRY_TEST_PYTHON:-/usr/bin/python3}"
LOAD_ROS=true
for argument in "$@"; do
  case "${argument}" in
    stop|status|--help|-h|--dry-run) LOAD_ROS=false ;;
  esac
done

if [[ "${LOAD_ROS}" == true ]]; then
  ROS_SETUP="${ODOMETRY_TEST_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
  [[ -f "${ROS_SETUP}" ]] || { printf 'ROS setup missing: %s\n' "${ROS_SETUP}" >&2; exit 1; }
  [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]] || {
    printf 'Workspace setup missing; build this workspace first.\n' >&2
    exit 1
  }
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
  export LD_LIBRARY_PATH="${WORKSPACE_ROOT}/src/orb_slam3/ORB_SLAM3/lib:${WORKSPACE_ROOT}/src/orb_slam3/deps/lib:${LD_LIBRARY_PATH:-}"
fi

exec "${PYTHON_COMMAND}" "${SCRIPT_DIR}/odometry_test.py" "$@"
