#!/usr/bin/env bash

# Process discovery and shutdown helpers for the navigation startup scripts.
# The caller provides WORKSPACE_ROOT, ROS_DISTRO_NAME, Pgrep/log/fail helpers.

read_process_argv() {
  local pid="$1"
  PROCESS_ARGV=()
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/cmdline" ]] || return 1
  mapfile -d '' -t PROCESS_ARGV <"/proc/${pid}/cmdline"
  ((${#PROCESS_ARGV[@]} > 0))
}

argv_has_sequence() {
  local -n arguments_ref="$1"
  shift
  local -a expected=("$@")
  local start
  local offset

  ((${#expected[@]} > 0 && ${#arguments_ref[@]} >= ${#expected[@]})) || return 1
  for ((start = 0; start + ${#expected[@]} <= ${#arguments_ref[@]}; ++start)); do
    for ((offset = 0; offset < ${#expected[@]}; ++offset)); do
      [[ "${arguments_ref[start + offset]}" == "${expected[offset]}" ]] || break
    done
    ((offset == ${#expected[@]})) && return 0
  done
  return 1
}

argv_contains_path() {
  local -n arguments_ref="$1"
  local expected_path
  expected_path="$(readlink -f -- "$2" 2>/dev/null)" || return 1
  local argument
  local resolved

  for argument in "${arguments_ref[@]}"; do
    [[ "${argument}" == */* ]] || continue
    resolved="$(readlink -f -- "${argument}" 2>/dev/null)" || continue
    [[ "${resolved}" == "${expected_path}" ]] && return 0
  done
  return 1
}

is_user_gui_pid() {
  local pid="$1"
  read_process_argv "${pid}" || return 1
  local -a gui_paths=(
    "${WORKSPACE_ROOT}/scripts/arena_route_frontend.py"
    "${WORKSPACE_ROOT}/scripts/arena_map_editor.py"
    "${WORKSPACE_ROOT}/scripts/start_arena_planner.sh"
    "${WORKSPACE_ROOT}/route_editor.py"
    "${WORKSPACE_ROOT}/src/visual_navigation/scripts/route_editor.py"
    "${WORKSPACE_ROOT}/install/visual_navigation/lib/visual_navigation/route_editor.py"
  )
  local path

  for path in "${gui_paths[@]}"; do
    if [[ -e "${path}" ]] && argv_contains_path PROCESS_ARGV "${path}"; then
      return 0
    fi
  done
  argv_has_sequence PROCESS_ARGV launch visual_navigation route_editor.launch.py ||
    argv_has_sequence PROCESS_ARGV run visual_navigation route_editor.py
}

is_arena_planner_pid() {
  local pid="$1"
  read_process_argv "${pid}" || return 1
  local backend="${WORKSPACE_ROOT}/install/arena_path_planner/lib/arena_path_planner/arena_planner_node"

  if [[ -e "${backend}" ]] && argv_contains_path PROCESS_ARGV "${backend}"; then
    return 0
  fi
  argv_has_sequence PROCESS_ARGV launch arena_path_planner arena_path_planner.launch.py
}

is_preserved_ros_pid() {
  is_user_gui_pid "$1" || is_arena_planner_pid "$1"
}

process_start_time() {
  local pid="$1"
  [[ -r "/proc/${pid}/stat" ]] || return 1
  awk '{print $22}' "/proc/${pid}/stat"
}

find_ros_processes() {
  local pattern
  pattern="/opt/ros/${ROS_DISTRO_NAME}/bin/ros2([[:space:]]|$)"
  pattern+="|/opt/ros/${ROS_DISTRO_NAME}/lib/[^[:space:]]+/[^[:space:]]+"
  pattern+="|${WORKSPACE_ROOT}/install/[^[:space:]]+/lib/[^[:space:]]+/[^[:space:]]+"
  "${PGREP_COMMAND}" -f "${pattern}" 2>/dev/null || true
}

stop_arena_planner_processes() {
  local -a discovered=()
  local -a targets=()
  local -a remaining=()
  local -A start_times=()
  local pid
  local attempt
  local current_start

  mapfile -t discovered < <(find_ros_processes)
  for pid in "${discovered[@]}"; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    is_arena_planner_pid "${pid}" || continue
    current_start="$(process_start_time "${pid}" 2>/dev/null)" || continue
    start_times["${pid}"]="${current_start}"
    targets+=("${pid}")
  done
  ((${#targets[@]} > 0)) || return 0

  log "Stopping ${#targets[@]} previous arena planner process(es)"
  kill -TERM "${targets[@]}" 2>/dev/null || true
  for attempt in {1..50}; do
    remaining=()
    for pid in "${targets[@]}"; do
      current_start="$(process_start_time "${pid}" 2>/dev/null)" || continue
      [[ "${current_start}" == "${start_times[${pid}]}" ]] || continue
      is_arena_planner_pid "${pid}" && remaining+=("${pid}")
    done
    ((${#remaining[@]} == 0)) && return 0
    sleep 0.1
  done

  log "Forcing ${#remaining[@]} previous arena planner process(es) to exit"
  kill -KILL "${remaining[@]}" 2>/dev/null || true
  for attempt in {1..10}; do
    targets=()
    for pid in "${remaining[@]}"; do
      current_start="$(process_start_time "${pid}" 2>/dev/null)" || continue
      [[ "${current_start}" == "${start_times[${pid}]}" ]] || continue
      is_arena_planner_pid "${pid}" && targets+=("${pid}")
    done
    ((${#targets[@]} == 0)) && return 0
    sleep 0.1
  done
  return 1
}

stop_existing_ros_nodes() {
  local -a discovered=()
  local -a targets=()
  local -a remaining=()
  local -a preserved=()
  local -A start_times=()
  local pid
  local attempt
  local current_start

  mapfile -t discovered < <(find_ros_processes)
  for pid in "${discovered[@]}"; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    if is_preserved_ros_pid "${pid}"; then
      preserved+=("${pid}")
      continue
    fi
    current_start="$(process_start_time "${pid}" 2>/dev/null)" || continue
    start_times["${pid}"]="${current_start}"
    targets+=("${pid}")
  done

  ((${#preserved[@]} == 0)) ||
    log "Keeping ${#preserved[@]} user GUI/planner process(es)"
  if ((${#targets[@]} == 0)); then
    log "No existing ROS 2 nodes found"
  else
    log "Stopping ${#targets[@]} existing ROS 2 processes"
    kill -INT "${targets[@]}" 2>/dev/null || true

    for attempt in {1..40}; do
      remaining=()
      for pid in "${targets[@]}"; do
        current_start="$(process_start_time "${pid}" 2>/dev/null)" || continue
        [[ "${current_start}" == "${start_times[${pid}]}" ]] || continue
        is_user_gui_pid "${pid}" || remaining+=("${pid}")
      done
      ((${#remaining[@]} == 0)) && break
      sleep 0.2
    done

    if ((${#remaining[@]} > 0)); then
      log "Forcing ${#remaining[@]} ROS 2 processes to exit"
      kill -TERM "${remaining[@]}" 2>/dev/null || true
      for attempt in {1..20}; do
        targets=()
        for pid in "${remaining[@]}"; do
          current_start="$(process_start_time "${pid}" 2>/dev/null)" || continue
          [[ "${current_start}" == "${start_times[${pid}]}" ]] || continue
          is_user_gui_pid "${pid}" || targets+=("${pid}")
        done
        ((${#targets[@]} == 0)) && break
        sleep 0.1
      done
      ((${#targets[@]} == 0)) || kill -KILL "${targets[@]}" 2>/dev/null || true
    fi
  fi

  "${ROS2_COMMAND}" daemon stop >/dev/null 2>&1 || true

  mapfile -t remaining < <(find_ros_processes)
  targets=()
  for pid in "${remaining[@]}"; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    is_preserved_ros_pid "${pid}" || targets+=("${pid}")
  done
  ((${#targets[@]} == 0)) ||
    fail "could not stop all existing ROS 2 processes: ${targets[*]}"
}
