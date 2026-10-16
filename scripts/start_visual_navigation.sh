#!/usr/bin/env bash

# Compatibility entry point.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/start_final_navigation.sh" "$@"
