#!/usr/bin/env bash
# Source from an entry point, then call profile_args "$@" before any side effect.
PI05_COMMON_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
export PI05_COMMON_DIR

profile_args() {
  PI05_PROFILE_ARGS=()
  while test "$#" -gt 0; do
    case "$1" in
      --robot-profile)
        test "$#" -ge 2 || { echo '--robot-profile needs a JSON path' >&2; return 2; }
        export PI05_ROBOT_PROFILE="$2"; shift 2 ;;
      *) PI05_PROFILE_ARGS+=("$1"); shift ;;
    esac
  done
  local task_profile_exports
  task_profile_exports=$(PYTHONDONTWRITEBYTECODE=1 python3 "$PI05_COMMON_DIR/robot_profile.py" --shell) || return
  eval "$task_profile_exports"
  case "$PI05_ROBOT_PROFILE" in
    *"'"*|*$'\n'*) echo 'Profile paths must not contain a single quote or newline' >&2; return 2 ;;
  esac
}

profile_bundle_digest() {
  sha256sum "$PI05_COMMON_DIR/robot_profile.py" "$PI05_COMMON_DIR/end_effector.py" \
    "$PI05_COMMON_DIR/profile.sh" "$PI05_ROBOT_PROFILE" | sha256sum | cut -c1-16
}

profile_install_common() {
  local task_peer=$1 task_destination=$2 task_expected task_actual
  ssh "${task_ssh[@]}" "$task_peer" "mkdir -p '$task_destination/common'"
  scp "${task_ssh[@]}" "$PI05_COMMON_DIR/robot_profile.py" "$PI05_COMMON_DIR/end_effector.py" \
    "$PI05_COMMON_DIR/profile.sh" "$task_peer:$task_destination/common/"
  scp "${task_ssh[@]}" "$PI05_ROBOT_PROFILE" "$task_peer:$task_destination/common/robot_profile.json"
  task_expected=$(sha256sum "$PI05_COMMON_DIR/robot_profile.py" "$PI05_COMMON_DIR/end_effector.py" \
    "$PI05_COMMON_DIR/profile.sh" "$PI05_ROBOT_PROFILE" | awk '{print $1}')
  task_actual=$(ssh "${task_ssh[@]}" "$task_peer" \
    "sha256sum '$task_destination/common/robot_profile.py' '$task_destination/common/end_effector.py' '$task_destination/common/profile.sh' '$task_destination/common/robot_profile.json'" | awk '{print $1}')
  test "$task_expected" = "$task_actual" || { echo 'Robot profile bundle checksum mismatch' >&2; return 3; }
}
