#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="${STATE_DIR:-/tmp/halmstad_ws}"
MODE_FILE="$STATE_DIR/run_mode.lock"
MODE_LOCK_ENABLE="${MODE_LOCK_ENABLE:-true}"
CURRENT_MODE="camera_stack"

is_launch_assignment() {
  [[ "${1:-}" == *":="* ]]
}

is_truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

acquire_mode_lock() {
  if ! is_truthy "$MODE_LOCK_ENABLE"; then
    return
  fi
  mkdir -p "$STATE_DIR"

  if [[ -f "$MODE_FILE" ]]; then
    local active_mode=""
    local active_pid=""
    read -r active_mode active_pid < "$MODE_FILE" || true
    if [[ -n "$active_pid" ]] && kill -0 "$active_pid" 2>/dev/null; then
      if [[ "$active_mode" != "$CURRENT_MODE" ]]; then
        echo "[run_uav_camera_stack] mode conflict: active mode '$active_mode' (pid=$active_pid). Stop it before starting '$CURRENT_MODE'."
        exit 21
      fi
      echo "[run_uav_camera_stack] '$CURRENT_MODE' already running (pid=$active_pid)."
      exit 22
    fi
    rm -f "$MODE_FILE"
  fi

  printf '%s %s\n' "$CURRENT_MODE" "$$" > "$MODE_FILE"
}

release_mode_lock() {
  if ! is_truthy "$MODE_LOCK_ENABLE"; then
    return
  fi
  if [[ -f "$MODE_FILE" ]]; then
    local active_mode=""
    local active_pid=""
    read -r active_mode active_pid < "$MODE_FILE" || true
    if [[ "$active_mode" == "$CURRENT_MODE" && "$active_pid" == "$$" ]]; then
      rm -f "$MODE_FILE"
    fi
  fi
}

REMAINING_ARGS=("$@")
WORLD="orchard"
UAV_NAME="dji1"
RUN_CONTROLLER="true"

if [[ "${#REMAINING_ARGS[@]}" -gt 0 ]] && ! is_launch_assignment "${REMAINING_ARGS[0]}"; then
  WORLD="${REMAINING_ARGS[0]}"
  REMAINING_ARGS=("${REMAINING_ARGS[@]:1}")
fi
if [[ "${#REMAINING_ARGS[@]}" -gt 0 ]] && ! is_launch_assignment "${REMAINING_ARGS[0]}"; then
  UAV_NAME="${REMAINING_ARGS[0]}"
  REMAINING_ARGS=("${REMAINING_ARGS[@]:1}")
fi
if [[ "${#REMAINING_ARGS[@]}" -gt 0 ]] && ! is_launch_assignment "${REMAINING_ARGS[0]}"; then
  RUN_CONTROLLER="${REMAINING_ARGS[0]}"
  REMAINING_ARGS=("${REMAINING_ARGS[@]:1}")
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

acquire_mode_lock
trap release_mode_lock EXIT INT TERM

set +e
ros2 launch lrs_halmstad run_uav_camera_stack.launch.py \
  world:="$WORLD" \
  uav_name:="$UAV_NAME" \
  run_controller:="$RUN_CONTROLLER" \
  "${REMAINING_ARGS[@]}"
STATUS=$?
set -e

trap - EXIT INT TERM
release_mode_lock
exit "$STATUS"
