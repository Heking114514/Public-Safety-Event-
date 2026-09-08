#!/usr/bin/env bash

# Shared lock for commands that write the workspace build/install/log trees.
# The caller must define WORKSPACE_ROOT before sourcing this file.

acquire_workspace_build_lock() {
  [[ -n "${WORKSPACE_ROOT:-}" ]] || return 2
  [[ -z "${WORKSPACE_BUILD_LOCK_FD:-}" ]] || return 2
  command -v flock >/dev/null 2>&1 || return 127

  local lock_file="${WORKSPACE_BUILD_LOCK_FILE:-${WORKSPACE_ROOT}/.workspace-build.lock}"
  exec {WORKSPACE_BUILD_LOCK_FD}>"${lock_file}" || return 1
  if ! flock "${WORKSPACE_BUILD_LOCK_FD}"; then
    exec {WORKSPACE_BUILD_LOCK_FD}>&-
    WORKSPACE_BUILD_LOCK_FD=""
    return 1
  fi
}

release_workspace_build_lock() {
  [[ "${WORKSPACE_BUILD_LOCK_FD:-}" =~ ^[0-9]+$ ]] || return 0
  flock -u "${WORKSPACE_BUILD_LOCK_FD}" 2>/dev/null || true
  exec {WORKSPACE_BUILD_LOCK_FD}>&-
  WORKSPACE_BUILD_LOCK_FD=""
}
