#!/usr/bin/env bash

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: build_workspace_common.sh ARCH [options]

ARCH must be x86_64 or arm64.

Options:
  --clean          Remove the workspace build/install/log directories first.
  --force-deps     Rebuild and reinstall Pangolin and Sophus.
  --skip-tests     Skip colcon test after the build.
  --no-install     Do not use apt; only for already-provisioned environments.
  --jobs COUNT     Parallel compile jobs; skips the interactive prompt.
  -h, --help       Show this help.
EOF
}

die() {
  printf '[build] ERROR: %s\n' "$*" >&2
  exit 1
}

warn() {
  printf '[build] WARNING: %s\n' "$*" >&2
}

log() {
  printf '[build] %s\n' "$*"
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }

EXPECTED_ARCH="$1"
shift
case "$EXPECTED_ARCH" in
  x86_64|arm64) ;;
  *) die "unsupported build target '$EXPECTED_ARCH'" ;;
esac

JOBS="${BUILD_JOBS:-}"
CLEAN_BUILD=false
FORCE_DEPS=false
RUN_TESTS=true
SKIP_INSTALL=false

while (($# > 0)); do
  case "$1" in
    --clean)
      CLEAN_BUILD=true
      shift
      ;;
    --force-deps)
      FORCE_DEPS=true
      shift
      ;;
    --skip-tests)
      RUN_TESTS=false
      shift
      ;;
    --no-install)
      SKIP_INSTALL=true
      shift
      ;;
    --jobs)
      (($# >= 2)) || die "--jobs requires a positive integer"
      JOBS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option '$1' (use --help)"
      ;;
  esac
done

CPU_COUNT="$(nproc 2>/dev/null || printf '1')"
[[ "$CPU_COUNT" =~ ^[1-9][0-9]*$ ]] || CPU_COUNT=1
DEFAULT_JOBS="$CPU_COUNT"
((DEFAULT_JOBS > 2)) && DEFAULT_JOBS=2

if [[ -z "$JOBS" ]]; then
  if [[ -t 0 && -t 1 ]]; then
    while true; do
      read -r -p "请输入编译线程数 (1-${CPU_COUNT}, 默认 ${DEFAULT_JOBS}): " JOBS_INPUT
      JOBS_INPUT="${JOBS_INPUT:-$DEFAULT_JOBS}"
      if [[ "$JOBS_INPUT" =~ ^[1-9][0-9]*$ ]] && ((JOBS_INPUT <= CPU_COUNT)); then
        JOBS="$JOBS_INPUT"
        break
      fi
      printf '[build] ERROR: 请输入 1 到 %s 之间的正整数\n' "$CPU_COUNT" >&2
    done
  else
    JOBS="$DEFAULT_JOBS"
    log "No interactive terminal; using ${JOBS} compile threads (use --jobs COUNT to choose explicitly)"
  fi
fi

[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || die "--jobs must be a positive integer"
((JOBS <= CPU_COUNT)) || die "--jobs cannot exceed the ${CPU_COUNT} logical CPUs available on this host"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_LOCK_HELPER="${SCRIPT_DIR}/build_lock.sh"
ORB_ROOT="${WORKSPACE_ROOT}/src/orb_slam3/ORB_SLAM3"
PANGOLIN_ROOT="${WORKSPACE_ROOT}/src/orb_slam3/Pangolin"
DEPS_ROOT="${ORB_SLAM3_DEPS_ROOT:-${WORKSPACE_ROOT}/src/orb_slam3/deps}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BUILD_ROOT="${WORKSPACE_ROOT}/build/${EXPECTED_ARCH}"

[[ -f "${WORKSPACE_ROOT}/.gitmodules" ]] || die "workspace git metadata is missing"
[[ -f "${ORB_ROOT}/CMakeLists.txt" ]] || die "ORB_SLAM3 submodule is missing"
[[ -f "${PANGOLIN_ROOT}/CMakeLists.txt" ]] || die "Pangolin submodule is missing"
[[ -f "${BUILD_LOCK_HELPER}" ]] || die "build lock helper is missing: ${BUILD_LOCK_HELPER}"
# shellcheck disable=SC1090
source "${BUILD_LOCK_HELPER}"

case "$EXPECTED_ARCH" in
  x86_64)
    case "$(uname -m)" in
      x86_64|amd64) ;;
      *) die "x86_build.sh must run on x86_64, found $(uname -m)" ;;
    esac
    FILE_ARCH_PATTERN='x86-64'
    ;;
  arm64)
    case "$(uname -m)" in
      aarch64|arm64) ;;
      *) die "arm_build.sh must run natively on ARM64, found $(uname -m)" ;;
    esac
    FILE_ARCH_PATTERN='ARM aarch64|ARM64'
    ;;
esac

# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || \
  die "Ubuntu 22.04 is required; found ${PRETTY_NAME:-unknown}"

if (( EUID == 0 )); then
  SUDO=()
elif command -v sudo >/dev/null 2>&1; then
  SUDO=(sudo)
else
  die "sudo is required when the script is not run as root"
fi

if (( EUID != 0 )) && [[ "$SKIP_INSTALL" == false ]]; then
  log "Sudo authorization is required to install missing dependencies"
  "${SUDO[@]}" -v || die "sudo authorization failed"
fi

APT_UPDATED=false

apt_update() {
  if [[ "$APT_UPDATED" == false ]]; then
    log "Updating apt package indexes"
    "${SUDO[@]}" apt-get update
    APT_UPDATED=true
  fi
}

is_deb_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | \
    grep -q 'install ok installed'
}

ensure_apt_packages() {
  local package
  local -a missing=()
  for package in "$@"; do
    if ! is_deb_installed "$package"; then
      missing+=("$package")
    fi
  done

  if ((${#missing[@]} == 0)); then
    log "All checked apt packages are already installed"
    return
  fi

  if [[ "$SKIP_INSTALL" == true ]]; then
    warn "Missing apt packages (installation disabled): ${missing[*]}"
    return
  fi

  apt_update
  log "Installing missing apt packages: ${missing[*]}"
  "${SUDO[@]}" apt-get install -y --no-install-recommends "${missing[@]}"
}

ensure_ros_repository() {
  if apt-cache show ros-humble-ros-base >/dev/null 2>&1; then
    return
  fi
  [[ "$SKIP_INSTALL" == false ]] || return

  ensure_apt_packages ca-certificates curl gnupg lsb-release software-properties-common
  if command -v add-apt-repository >/dev/null 2>&1; then
    "${SUDO[@]}" add-apt-repository --yes universe >/dev/null
  fi

  local key_tmp
  key_tmp="$(mktemp)"
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor >"${key_tmp}"
  "${SUDO[@]}" install -D -m 0644 "${key_tmp}" \
    /usr/share/keyrings/ros-archive-keyring.gpg
  rm -f "${key_tmp}"

  local codename="${VERSION_CODENAME:-jammy}"
  local dpkg_arch
  dpkg_arch="$(dpkg --print-architecture)"
  printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
    "$dpkg_arch" "$codename" | \
    "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null

  APT_UPDATED=false
  apt_update
}

ensure_ros() {
  if [[ "$SKIP_INSTALL" == true && ! -r "$ROS_SETUP" ]]; then
    die "ROS 2 Humble is missing and --no-install was requested"
  fi
  ensure_ros_repository
  ensure_apt_packages \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-robot-localization

  [[ -r "$ROS_SETUP" ]] || die "ROS 2 Humble setup was not installed at $ROS_SETUP"
  # ROS setup scripts use variables that may be unset under `set -u`.
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u

  if [[ "$SKIP_INSTALL" == false && ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep sources"
    "${SUDO[@]}" rosdep init || warn "rosdep init returned a non-zero status"
  fi
  if ! rosdep update; then
    warn "rosdep update failed; using the existing rosdep cache"
  fi

  if [[ "$SKIP_INSTALL" == true ]]; then
    log "Skipping rosdep install because --no-install was requested"
    return
  fi

  log "Installing ROS dependencies declared by the workspace"
  rosdep install \
    --from-paths "${WORKSPACE_ROOT}/src" \
    --ignore-src \
    --rosdistro humble \
    --skip-keys 'Pangolin realsense2_camera' \
    -r -y
}

try_install_optional() {
  local package="$1"
  if ! apt-cache show "$package" >/dev/null 2>&1; then
    warn "Optional package $package is not available for this platform"
    return 0
  fi
  if is_deb_installed "$package"; then
    log "Optional package $package is already installed"
    return 0
  fi
  if [[ "$SKIP_INSTALL" == true ]]; then
    warn "Optional package $package is missing (installation disabled)"
    return 0
  fi
  apt_update
  if ! "${SUDO[@]}" apt-get install -y --no-install-recommends "$package"; then
    warn "Could not install optional package $package; compilation can continue"
  fi
}

ensure_runtime_packages() {
  ensure_apt_packages \
    build-essential cmake g++ git pkg-config file \
    libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
    libglew-dev libgl1-mesa-dev libglu1-mesa-dev \
    libegl1-mesa-dev libgles2-mesa-dev \
    libwayland-dev libxkbcommon-dev wayland-protocols \
    libx11-dev libxrandr-dev libxi-dev libxxf86vm-dev \
    libjpeg-dev libpng-dev libtiff-dev liblz4-dev libzstd-dev \
    python3-dev python3-numpy python3-tk python3-yaml

  ensure_ros

  # The RealSense ROS driver is runtime-only for this build. It is optional
  # here because official ARM packages are not available for every board.
  try_install_optional ros-humble-realsense2-camera
  try_install_optional librealsense2-dev
  try_install_optional librealsense2-utils

  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
  if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
    warn "realsense2_camera is not installed; build it from the ARM-compatible librealsense/realsense-ros sources before using the D455 launch file"
  fi
}

prepare_sources() {
  log "Initializing git submodules"
  git -C "$WORKSPACE_ROOT" submodule update --init --recursive

  if [[ ! -f "${ORB_ROOT}/Vocabulary/ORBvoc.txt" ]]; then
    [[ -f "${ORB_ROOT}/Vocabulary/ORBvoc.txt.tar.gz" ]] || \
      die "ORB vocabulary archive is missing"
    log "Extracting ORB-SLAM3 vocabulary"
    tar -xzf "${ORB_ROOT}/Vocabulary/ORBvoc.txt.tar.gz" \
      -C "${ORB_ROOT}/Vocabulary"
  fi

  mkdir -p "$DEPS_ROOT" "$BUILD_ROOT"
}

file_matches_native_arch() {
  local file_path="$1"
  [[ -f "$file_path" ]] || return 1
  file -b "$file_path" 2>/dev/null | grep -Eq "$FILE_ARCH_PATTERN"
}

build_pangolin() {
  local config_file="${DEPS_ROOT}/lib/cmake/Pangolin/PangolinConfig.cmake"
  local core_library="${DEPS_ROOT}/lib/libpango_core.so"
  local needs_build=false

  if [[ "$FORCE_DEPS" == true ]] || [[ ! -f "$config_file" ]] || \
     ! file_matches_native_arch "$core_library"; then
    needs_build=true
  fi
  if [[ "$EXPECTED_ARCH" == arm64 ]] && \
     rg -q '/usr/lib/x86_64-linux-gnu' "${DEPS_ROOT}/lib/cmake/Pangolin" 2>/dev/null; then
    needs_build=true
  fi

  if [[ "$needs_build" == false ]]; then
    log "Using the existing ${EXPECTED_ARCH} Pangolin installation"
    return
  fi

  log "Building Pangolin for ${EXPECTED_ARCH}"
  local build_dir="${BUILD_ROOT}/pangolin"
  cmake -S "$PANGOLIN_ROOT" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$DEPS_ROOT" \
    -DBUILD_TOOLS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build "$build_dir" --parallel "$JOBS"
  cmake --install "$build_dir"
  file_matches_native_arch "$core_library" || \
    die "Pangolin was built, but $core_library is not $EXPECTED_ARCH"
}

build_sophus() {
  local config_file="${DEPS_ROOT}/share/sophus/cmake/SophusConfig.cmake"
  if [[ "$FORCE_DEPS" == false && -f "$config_file" ]]; then
    log "Using the existing Sophus installation"
    return
  fi

  log "Building Sophus"
  local build_dir="${BUILD_ROOT}/sophus"
  cmake -S "${ORB_ROOT}/Thirdparty/Sophus" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$DEPS_ROOT" \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build "$build_dir" --parallel "$JOBS"
  cmake --install "$build_dir"
}

build_native_library() {
  local source_dir="$1"
  local build_dir="$2"
  shift 2
  cmake -S "$source_dir" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    "$@"
  cmake --build "$build_dir" --parallel "$JOBS"
}

build_orbslam3() {
  log "Building DBoW2"
  build_native_library \
    "${ORB_ROOT}/Thirdparty/DBoW2" \
    "${BUILD_ROOT}/dbow2"
  [[ -f "${ORB_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so" ]] || \
    die "DBoW2 library was not generated"

  log "Building g2o"
  build_native_library \
    "${ORB_ROOT}/Thirdparty/g2o" \
    "${BUILD_ROOT}/g2o" \
    -DCMAKE_CXX_STANDARD=14
  [[ -f "${ORB_ROOT}/Thirdparty/g2o/lib/libg2o.so" ]] || \
    die "g2o library was not generated"

  log "Building ORB-SLAM3"
  local build_dir="${BUILD_ROOT}/orbslam3"
  cmake -S "$ORB_ROOT" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DPangolin_DIR="${DEPS_ROOT}/lib/cmake/Pangolin"
  cmake --build "$build_dir" --target ORB_SLAM3 --parallel "$JOBS"
  [[ -f "${ORB_ROOT}/lib/libORB_SLAM3.so" ]] || \
    die "ORB-SLAM3 library was not generated"
}

build_ros_workspace() {
  log "Building ROS 2 packages"
  cd "$WORKSPACE_ROOT"
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" colcon build \
    --symlink-install \
    --executor sequential \
    --packages-ignore ORB_SLAM3 pangolin \
    --cmake-args \
      -DSophus_DIR="${DEPS_ROOT}/share/sophus/cmake" \
      -DPangolin_DIR="${DEPS_ROOT}/lib/cmake/Pangolin" \
      -DORB_SLAM3_ROOT_DIR="$ORB_ROOT"
}

check_library_dependencies() {
  local library_path
  local missing
  local library_path_env="${ORB_ROOT}/lib:${ORB_ROOT}/Thirdparty/DBoW2/lib:${ORB_ROOT}/Thirdparty/g2o/lib:${DEPS_ROOT}/lib:${LD_LIBRARY_PATH:-}"
  for library_path in \
    "${ORB_ROOT}/lib/libORB_SLAM3.so" \
    "${ORB_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so" \
    "${ORB_ROOT}/Thirdparty/g2o/lib/libg2o.so" \
    "${DEPS_ROOT}/lib/libpango_core.so"; do
    file_matches_native_arch "$library_path" || \
      die "library has the wrong architecture: $library_path"
    missing="$(LD_LIBRARY_PATH="$library_path_env" ldd "$library_path" 2>&1 | grep 'not found' || true)"
    [[ -z "$missing" ]] || die "unresolved runtime dependency for $library_path: $missing"
  done
}

run_tests() {
  [[ "$RUN_TESTS" == true ]] || return 0
  log "Running colcon tests"
  cd "$WORKSPACE_ROOT"
  colcon test \
    --executor sequential \
    --packages-ignore ORB_SLAM3 pangolin \
    --event-handlers console_direct+
  colcon test-result --verbose
}

if [[ "$CLEAN_BUILD" == true ]]; then
  log "Removing generated workspace directories"
  rm -rf -- "${WORKSPACE_ROOT}/build" "${WORKSPACE_ROOT}/install" "${WORKSPACE_ROOT}/log"
fi

log "Preparing ${EXPECTED_ARCH} build on $(uname -m)"
ensure_runtime_packages
log "Waiting for exclusive workspace build access"
acquire_workspace_build_lock || die "could not acquire the workspace build lock"
trap release_workspace_build_lock EXIT
prepare_sources
build_pangolin
build_sophus
build_orbslam3
build_ros_workspace
check_library_dependencies
run_tests
release_workspace_build_lock
trap - EXIT

log "Build completed successfully"
log "For later ROS package changes: source ${ROS_SETUP} && source install/setup.bash && colcon build"
log "Changes inside ORB_SLAM3 or Pangolin require rerunning the native-library stage"
log "The D455 launch additionally requires an ARM-compatible realsense2_camera installation"
